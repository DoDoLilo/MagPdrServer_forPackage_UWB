from enum import Enum
import os.path


# 文件名后缀枚举类
class FileEnd(Enum):
    CSV = ".csv"
    PT = ".pt"
    NPY = ".npy"
    TXT = ".txt"


# 判断文件名是否以目标后缀结尾
def file_name_end_with(file_name, end):
    if file_name is None:
        return False

    try:
        file_name = str(file_name)
        end = str(end)
        return file_name.endswith(end)
    except Exception as e:
        print(e)
        return False


# 文件是否存在
def file_is_exist(file_name):
    return os.path.exists(file_name)
