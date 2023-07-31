import h264decoder
import numpy as np

f = open('frame_data.npy', 'rb')
decoder = h264decoder.H264Decoder()
while 1:
  data_in = f.read(1024)
  if not data_in:
    break
  framedatas = decoder.decode(data_in)
  for framedata in framedatas:
    (frame, w, h, ls) = framedata
    if frame is not None:
        #print('frame size %i bytes, w %i, h %i, linesize %i' % (len(frame), w, h, ls))
        frame = np.frombuffer(frame, dtype=np.ubyte, count=len(frame))
        frame = frame.reshape((h, ls//3, 3))
        frame = frame[:,:w,:]
        # At this point `frame` references your usual height x width x rgb channels numpy array of unsigned bytes.