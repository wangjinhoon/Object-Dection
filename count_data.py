import os


fileDir = r"C:\Users\user\Downloads\tstl\JPEGImages (1)"
fileExt = r".txt"
txt_lst = [_ for _ in os.listdir(fileDir) if _.endswith(fileExt)]

dic = {}

for file_name in txt_lst:
    with open(os.path.join(fileDir, file_name), 'r', encoding='UTF-8', errors='ignores') as f:
        classes = [i.split(' ')[0] for i in f.readlines()]
        for cls in classes:
            if cls not in dic:
                dic[cls] = 1
            else:
                dic[cls] += 1

ls = sorted(dic.items())

print(ls)