from sklearn.model_selection import train_test_split
import os


# path to dataset
dataset_dir = "C:\\Users\\user\\Downloads\\tstl\\tstl_dataset"
all_dir = dataset_dir + "\\all"
image_lst = [_ for _ in os.listdir(all_dir + "\\JPEGimages")]


def mkdir_if_not_exists(*paths):
    for path in paths:
        if not os.path.exists(path):
            os.mkdir(path)


def rename_datasest():
    global all_dir, image_lst
    for i, file_name in enumerate(image_lst):
        img_path = os.path.join(all_dir + "\\JPEGimages", file_name)
        anno_path = os.path.join(all_dir + "\\annotations", file_name.split('.')[0] + ".txt")
        if not os.path.exists(anno_path):
            # create empty txt file
            with open(anno_path, 'w', encoding='UTF-8', errors='ignores') as f:
                pass
        new_file_name = str(i).zfill(4)
        os.rename(img_path, os.path.join(all_dir + "\\JPEGimages", new_file_name + ".png"))
        os.rename(anno_path, os.path.join(all_dir + "\\annotations", new_file_name + ".txt"))
    image_lst = [_ for _ in os.listdir(all_dir + "\\JPEGimages")]


def split_dataset():
    global dataset_dir, all_dir, image_lst
    train, test = train_test_split(image_lst, train_size=0.8, test_size=0.2, random_state=1)
    print("train:", len(train), "test:", len(test))

    all_img_dir = all_dir + "\\JPEGimages"
    all_anno_dir = all_dir + "\\annotations"

    train_dir = dataset_dir + "\\train"
    img_dir = train_dir + "\\JPEGimages"
    anno_dir = train_dir + "\\annotations"
    set_dir = train_dir + "\\imagesets"

    mkdir_if_not_exists(img_dir, anno_dir, set_dir)

    with open(set_dir + "\\train.txt", 'w', encoding='UTF-8', errors='ignores') as f:
        for train_data in train:
            file_name = train_data.split('.')[0]
            os.rename(os.path.join(all_img_dir, train_data), os.path.join(img_dir, train_data))
            os.rename(os.path.join(all_anno_dir, file_name + ".txt"), os.path.join(anno_dir, file_name + ".txt"))
            f.write(file_name + "\n")

    test_dir = dataset_dir + "\\test"
    img_dir = test_dir + "\\JPEGimages"
    anno_dir = test_dir + "\\annotations"
    set_dir = test_dir + "\\imagesets"

    mkdir_if_not_exists(img_dir, anno_dir, set_dir)

    with open(set_dir + "\\test.txt", 'w', encoding='UTF-8', errors='ignores') as f:
        for test_data in test:
            file_name = test_data.split('.')[0]
            os.rename(os.path.join(all_img_dir, test_data), os.path.join(img_dir, test_data))
            os.rename(os.path.join(all_anno_dir, file_name + ".txt"), os.path.join(anno_dir, file_name + ".txt"))
            f.write(file_name + "\n")


def main():
    rename_datasest()
    split_dataset()


if __name__ == "__main__":
    main()
