echo 'creating link, this requires superuser priviliges'
sudo ln -s ~/dd2419_ws/src/course_packages/dd2419_resources/worlds_json ~/dd2419_ws/src/crazyflie_packages/milestone3/

echo 'copying files'
cp ./* ../worlds_json/ && rm ../worlds_json/map.bash || echo 'the link was not created properly, please ask Ivan what to do.'

echo 'appending the link to git-ignore'
echo 'worlds_json' >> ~/dd2419_ws/src/crazyflie_packages/.git/info/exclude
