echo $1

while true; do 
  inotifywait -r -e modify,create,delete /home/abobco/RaspberryPi/OCV_Projects
  # rsync -avze "ssh -i /home/abobco/.ssh/id_rsa" /home/abobco/RaspberryPi/OCV_Projects pi@$1:/home/pi/shared/novfp/RaspberryPi
  rsync -avze "ssh -i /home/abobco/.ssh/id_rsa" /home/abobco/RaspberryPi/OCV_Projects pi@$1:/home/pi/
done
