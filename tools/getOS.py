import os


def detect_os():
    if os.name == 'nt':
        return 'Windows'
    elif os.name == 'posix':
        return 'Linux/Unix'
    else:
        return 'Unknown'