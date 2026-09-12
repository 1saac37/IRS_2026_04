import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/workspace/irs_ws/src/IRS_2026_04/install/pa_warehouse_status'
