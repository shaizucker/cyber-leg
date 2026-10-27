# P-T-M settings matching the working 0-to-105-degree GUI.
Set-Location -LiteralPath $PSScriptRoot
python -u knee_bridge.py --kp-pos 20 --kd-pos 4 @args
