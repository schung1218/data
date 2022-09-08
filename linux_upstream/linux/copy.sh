#git status | grep modified | awk '{print $2}' > list.txt
mkdir -p linux-temp
xargs -a ./list.txt cp -rf --parents -t ./linux-temp

