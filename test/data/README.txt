-In order to extract jpeg frames from a mjpeg file, use the following command (assumes you already have ffmpeg installed):
	$ ffmpeg -i file.mjpeg -vcodec mjpeg -f image2 frame%d.jpg
