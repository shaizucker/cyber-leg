"""LAN HTTP access keeps exact Host, Origin and token validation; no hardware."""
import json
import threading
import unittest
from unittest.mock import Mock
from urllib.request import Request, urlopen
from urllib.error import HTTPError
from knee_bridge import BridgeServer, make_handler


class LANAccessTests(unittest.TestCase):
    def setUp(self):
        self.bridge = Mock()
        self.bridge.status.return_value = {'armed': False}
        self.server = BridgeServer(('127.0.0.1',0), make_handler(self.bridge,'test',0))
        self.port = self.server.server_port
        self.host = f'10.0.0.22:{self.port}'
        self.server.RequestHandlerClass = make_handler(self.bridge,'test',self.port,'10.0.0.22')
        self.thread = threading.Thread(target=self.server.serve_forever,daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def request(self, route, payload=None, **overrides):
        headers = {'Host':self.host,'Origin':f'http://{self.host}',
                   'X-Bridge-Token':'test','Content-Type':'application/json',**overrides}
        return urlopen(Request(f'http://127.0.0.1:{self.port}/api/{route}',
                              data=None if payload is None else json.dumps(payload).encode(),headers=headers))

    def test_exact_lan_host_can_get_session_and_update(self):
        with self.request('session') as response:
            self.assertEqual(json.load(response)['token'],'test')
        with self.request('update',{'angle_deg':0,'running':False}) as response:
            self.assertFalse(json.load(response)['armed'])
        self.bridge.update.assert_called_once()

    def test_foreign_host_origin_and_token_cannot_control(self):
        for overrides in ({'Host':f'other.local:{self.port}'}, {'Origin':'http://other.local'},
                          {'X-Bridge-Token':'wrong'}):
            with self.subTest(overrides=overrides), self.assertRaises(HTTPError) as error:
                self.request('update',{'angle_deg':0,'running':False},**overrides)
            self.assertEqual(error.exception.code,403)
        self.bridge.update.assert_not_called()
        with self.assertRaises(HTTPError) as error:
            self.request('session',Host=f'other.local:{self.port}')
        self.assertEqual(error.exception.code,403)


if __name__ == '__main__':
    unittest.main()
