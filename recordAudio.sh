ShowUsage()
{
	echo
	echo 'USB Audio Recorder For Time Synced Recordings Script'
	echo 'Contact: Daniel M. Lofaro - dan@danLofaro.com'
	echo 'Usage:'
	echo '-n         - File name'
	echo '             Example: recordAudio.sh -n robot '
	echo '             Resulting File: robot.wav' 
	echo
}
case "$1" in
	'-n' )
		arecord -D hw:1,0 -f cd -c1 $2.wav
	;;
	* )
		ShowUsage
		exit 1
	;;
esac
exit 0
