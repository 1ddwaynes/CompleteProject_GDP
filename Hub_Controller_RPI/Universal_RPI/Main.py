import _init_GPS
import _init_Serial


def check_val(data, what):
    if b"what" in data:
        return False
    else:
        return True


def main():
    pass