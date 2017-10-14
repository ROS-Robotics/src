#!/bin/bash
OK=0

while [  true ]; do
	echo 'cheking...'
	REM=$(netstat -an | grep -E "\:22[ \t]+" | grep ESTABLISHED | wc -l)
	echo $REM
	if [ "$REM" = "1" ]; then
		if [ "$OK" = "0" ]; then
		espeak "SSH client connection ESTABLISHED" -a 200  -ven-us+f2 -s170
		OK=1
		fi
	else
		if [ "$OK" = "1" ]; then
		espeak "SSH client ended" -a 200  -ven-us+f2 -s170
		OK=0
		fi
	fi

	sleep 5
done
