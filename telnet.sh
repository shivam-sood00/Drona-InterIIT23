#!/bin/bash
{ echo "+++AT MODE 3"; sleep 1; echo "+++AT STA drago 1234abcd"; sleep 1; echo "+++AT MODE 1"; sleep 1; } | telnet 192.168.4.1