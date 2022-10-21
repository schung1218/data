1. Playback

aplay –c 2 –f S16_LE 8k2ch.pcm

 
2. Record then playback

arecord –d 10 –f S16_LE –c2 –r8000 –t wav –D plughw:0,0 test.pcm
aplay –c 2 –f S16_LE test.pcm

 
