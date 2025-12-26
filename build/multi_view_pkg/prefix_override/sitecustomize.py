import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/affan/Documents/FY_Project/install/multi_view_pkg'
