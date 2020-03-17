# generate files warn_left.txt and warn_left.names warn_left.data in ~/yolov3/data/
#This script is part of preperation for training the yolo network
TRAINDIR=~/yolov3
echo "This script finds all images and labels and generates a file in $TRAINDIR/data/ to point to our data"

DATADIR=~/Traning_set


FILES=$(find $DATADIR -wholename "**/images/train/*")
rm $TRAINDIR/data/warn_left_train.txt
touch $TRAINDIR/data/warn_left_train.txt
for FILE in $FILES; do
  echo $FILE >> $TRAINDIR/data/warn_left_train.txt
done

FILES=$(find $DATADIR -wholename "**/images/val/*")
rm $TRAINDIR/data/warn_left_val.txt
touch $TRAINDIR/data/warn_left_val.txt
for FILE in $FILES; do
  echo $FILE >> $TRAINDIR/data/warn_left_val.txt
done


echo 'warn_left' > $TRAINDIR/data/warn_left.names

echo 'classes=2
train=data/warn_left_train.txt
valid=data/warn_left_val.txt
names=data/warn_left.names
backup=backup/
eval=warn_left' > $TRAINDIR/data/warn_left.data

echo 'do not forget to edit the cfg file'
echo 'change classes=n
number of filters in conv.-layer before yolo-layer to (5+n)*3'
