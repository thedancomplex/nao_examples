NAME=$1
arecord -D hw:1,0 -f cd -c1 $NAME.wav
