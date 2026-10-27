"""Cancelled HTTP connections must not be retried as application errors."""
import io
import json
import unittest
from email.message import Message
from unittest.mock import Mock, patch
from http.server import SimpleHTTPRequestHandler

from knee_bridge import KneeBridge, make_handler
from test_knee_bridge import FakeMotor


class FailingWriter(io.BytesIO):
    def __init__(self, failure, at_write):
        super().__init__()
        self.failure,self.at_write=failure,at_write
        self.writes=0

    def write(self,data):
        self.writes+=1
        if self.writes==self.at_write:
            raise self.failure('client disconnected')
        return super().write(data)


class HTTPDisconnectTests(unittest.TestCase):
    def setUp(self):
        self.motor=FakeMotor()
        self.bridge=KneeBridge(lambda:self.motor)
        self.handler_type=make_handler(self.bridge,'test',8000)

    def tearDown(self):
        self.bridge.close()

    def handler(self,payload=None):
        h=object.__new__(self.handler_type)
        h.request_version='HTTP/1.1'
        h.command='POST'
        h.path='/api/update'
        h.requestline='POST /api/update HTTP/1.1'
        h.close_connection=False
        h.client_address=('127.0.0.1',12345)
        h.headers=Message()
        for name,value in {'Host':'127.0.0.1:8000','Origin':'http://127.0.0.1:8000',
                           'Content-Type':'application/json','X-Bridge-Token':'test'}.items():
            h.headers[name]=value
        data=json.dumps(payload if payload is not None else {'angle_deg':10,'running':False}).encode()
        h.headers['Content-Length']=str(len(data))
        h.rfile=io.BytesIO(data)
        h.wfile=io.BytesIO()
        h.send_response=Mock(wraps=h.send_response)
        return h

    def test_aborted_success_reply_never_attempts_a_second_response(self):
        for error in (ConnectionAbortedError,ConnectionResetError,BrokenPipeError):
            for at in (1,2):
                with self.subTest(error=error.__name__,at_write=at):
                    h=self.handler()
                    h.wfile=FailingWriter(error,at)
                    h.do_POST()
                    self.assertTrue(h.close_connection)
                    self.assertEqual([c.args[0] for c in h.send_response.call_args_list],[200])
                    self.assertAlmostEqual(self.bridge.goal,10*3.141592653589793/180)
        self.assertEqual(self.motor.calls,[])

    def test_aborted_error_reply_is_not_retried_as_500(self):
        h=self.handler({'angle_deg':-1,'running':False})
        h.wfile=FailingWriter(ConnectionAbortedError,1)
        h.do_POST()
        self.assertEqual([c.args[0] for c in h.send_response.call_args_list],[400])
        self.assertTrue(h.close_connection)
        self.assertIsNone(self.bridge.goal)

    def test_aborted_request_body_does_not_dispatch_or_reply(self):
        h=self.handler()
        h.rfile=Mock()
        h.rfile.read.side_effect=ConnectionResetError('client closed during upload')
        h.do_POST()
        h.send_response.assert_not_called()
        self.assertTrue(h.close_connection)
        self.assertIsNone(self.bridge.goal)

    def test_get_disconnect_is_handled(self):
        h=self.handler()
        h.command='GET'
        h.path='/api/session'
        h.wfile=FailingWriter(BrokenPipeError,2)
        h.do_GET()
        self.assertTrue(h.close_connection)
        self.assertEqual([c.args[0] for c in h.send_response.call_args_list],[200])

    def test_request_parser_or_static_response_disconnect_is_handled(self):
        h=self.handler()
        with patch.object(SimpleHTTPRequestHandler,'handle_one_request',side_effect=ConnectionAbortedError('closed')):
            h.handle_one_request()
        self.assertTrue(h.close_connection)

    def test_real_control_error_still_returns_500(self):
        h=self.handler()
        with patch.object(self.bridge,'update',side_effect=RuntimeError('control failed')):
            h.do_POST()
        self.assertEqual([c.args[0] for c in h.send_response.call_args_list],[500])
        self.assertIn(b'control failed',h.wfile.getvalue())
        self.assertFalse(h.close_connection)


if __name__=='__main__':
    unittest.main()
