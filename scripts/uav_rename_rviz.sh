#!/bin/bash

RVIZ_CFG_PATH=$1

RVIZ_CFG_FILENAME=`basename "$RVIZ_CFG_PATH"`
RVIZ_CFG_NEWPATH="/tmp/$RVIZ_CFG_FILENAME.renamed.rviz"

if [ ! -f $RVIZ_CFG_PATH ]; then

  echo Rviz config file \"$RVIZ_CFG_PATH\" does not exist! >&2
  exit 1

fi

cp "$RVIZ_CFG_PATH" "$RVIZ_CFG_NEWPATH"

sed -i "s/uav[0-9]*/$UAV_NAME/g" "$RVIZ_CFG_NEWPATH"

rviz -d "$RVIZ_CFG_NEWPATH"
