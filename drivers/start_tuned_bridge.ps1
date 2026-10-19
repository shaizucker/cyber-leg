# Bench-tuned 5-to-7-degree profile; baseline defaults remain in knee_bridge.py.
Set-Location -LiteralPath $PSScriptRoot
python -u knee_bridge.py --kp-pos 40 --kd-pos 10 --kp-vel 50 --kd-vel 0 --ki-vel 0.001
