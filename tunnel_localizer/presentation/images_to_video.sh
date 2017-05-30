# avconv -r FPS -i frame%04d.jpg -b:v BITRATE OUTPUT.mp4
avconv -y -r 30 -i images/panaromic_%05d.png -b:v 120000k panaromic.mp4

