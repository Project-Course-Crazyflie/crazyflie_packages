#This script is part of preperation for YOLO network training.
#It changes the first component in all label files to fit 
TO=0
FILES=$(find . -name image*.txt)
declare -a TXT
for FILE in $FILES; do
  echo "before: " $(cat $FILE)
  TXT=($(cat $FILE))
  TXT[0]=$TO
  echo ${TXT[@]} > $FILE
  echo "after: " $(cat $FILE)
done
