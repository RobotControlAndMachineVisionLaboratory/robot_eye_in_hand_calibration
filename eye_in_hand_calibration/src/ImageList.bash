#!/bin/bash
for arg in $( seq 1 70 )
do
	echo "inputRGB${arg}.png" >> names.txt
done
