ipcs -s | egrep '0x*' | sed 's/0x[0-9a-f]* //' | egrep -o '[0-9]*' | xargs -n1 ipcrm -s
ipcs -m | egrep '0x*' | sed 's/0x[0-9a-f]* //' | egrep -o '[0-9]*' | xargs -n1 ipcrm -m
