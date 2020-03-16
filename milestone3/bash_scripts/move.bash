#This is a script that moves all images and label files from one directory into image train/var and label train/var subdirectories
#It is strictly to prepare for training the YOLO network

echo THIS SCRIPT DOES NOT SUPPORT IMAGES WITHOUT LABEL FILES
TODIR=./images/
IMGFORMAT=image*.png
LBLFORMAT=image*.txt
VALRATIO=10

IMGS=$(find . -maxdepth 1 -name "$IMGFORMAT" | sort )


count=0
for file in $IMGS; do
  let count++
  if [ $count -eq $VALRATIO ]; then
    cp $file "$TODIR/val"
    let count=0
  else
    cp $file "$TODIR/train"
  fi
done

TODIR=./labels
LBLS=$(find . -maxdepth 1 -name "$LBLFORMAT" | sort)
count=0
for file in $LBLS; do
  let count++
  if [ $count -eq $VALRATIO ]; then
    cp $file "$TODIR/val"
    let count=0
  else
    cp $file "$TODIR/train"
  fi
done
