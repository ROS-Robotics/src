#!/bin/bash

sleep 10

myip=
while IFS=$': \t' read -a line ;do
    [ -z "${line%inet}" ] && ip=${line[${#line[1]}>4?1:2]} &&
        [ "${ip#127.0.0.1}" ] && myip=$ip
  done< <(LANG=C /sbin/ifconfig wlan0)


if !([ -z "$myip" ]); then
espeak "Wireless Network I P: $myip" -a 200  -ven-us+f2 -s170
fi

myip=
while IFS=$': \t' read -a line ;do
    [ -z "${line%inet}" ] && ip=${line[${#line[1]}>4?1:2]} &&
        [ "${ip#127.0.0.1}" ] && myip=$ip
  done< <(LANG=C /sbin/ifconfig eth0)

if !([ -z "$myip" ]); then
espeak "Ethernet Network I P: $myip" -a 200  -ven-us+f2 -s170
fi

