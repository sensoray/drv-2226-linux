#!/usr/bin/python

# Video4Linux Capture Demo in python
# Copyright (c) 2010-2012 Sensoray Co., Inc.


import pygtk
pygtk.require('2.0')
import gtk, os, ctypes, glob, threading, subprocess, time, select, string, sys, socket
from v4l2 import *
from fcntl import *
from mmap import *


V4L2_PIX_FMT_MP4V = 0x5634504D
V4L2_PIX_FMT_H264 = 0x34363248
V4L2_PIX_FMT_MP42 = 0x3234504D
V4L2_PIX_FMT_MP2V = 0x5632504D
V4L2_MPEG_VIDEO_ENCODING_MPEG_4 = 3

debug = False
for arg in sys.argv:
	if arg == "-d":
		debug = True


def dprint(*args):
	if debug:
		for x in args:
			print x,
		print

def make_menu_item(name, callback, data=None, value=None):
	item = gtk.MenuItem(name)
	item.value = value
	item.connect("activate", callback, data)
	item.show()
	return item

def format_k(scale, value):
	if value >= 1000000:
		return "%.0d,%03.0dK" % (value / 1000000, (value / 1000) % 1000)
	if value >= 100000:
		return "%.0dK" % (value / 1000)
	return "%d" % value

class MplayerThread(threading.Thread):
	def __init__(self, name, outfmt, width, height, norm, input, demo):
		threading.Thread.__init__(self)
		self.devname = name
		self.outfmt = outfmt
		self.width = width
		self.height = height
		self.norm = norm
		self.input = input
		self.action = "Previewing"
		self.demo = demo
	def run(self):
		cmd = ""
		if self.outfmt == V4L2_PIX_FMT_NV12:
			cmd += "outfmt=nv12:"
		elif self.outfmt == V4L2_PIX_FMT_YUYV:
			cmd += "outfmt=yuy2:"
		elif self.outfmt == V4L2_PIX_FMT_UYVY:
			cmd += "outfmt=uyvy:"
		elif self.outfmt == V4L2_PIX_FMT_RGB24:
			cmd += "outfmt=rgb24:"
		elif self.outfmt == V4L2_PIX_FMT_RGB565:
			cmd += "outfmt=rgb16:"
		elif self.outfmt != None: cmd += "outfmt=0x%x:" % self.outfmt
		if self.width != None: cmd += "width=%d:" % self.width
		if self.height != None: cmd += "height=%d:" % self.height
		if self.norm != None: cmd += "norm=%s:" % self.norm
		if self.devname != None: cmd += "device=%s:" % self.devname
		if self.input != None: cmd += "input=%s:" % self.input
		if cmd != "": cmd = " -tv " + cmd
		if debug: cmd += " -quiet"
		else: cmd += " -really-quiet"
		cmd = "mplayer tv://" + cmd
		dprint(cmd)
		try: self.p = subprocess.Popen(cmd, stdin=subprocess.PIPE, shell=True)
		except: self.demo.message_box("Could not open mplayer")
		try: self.p.wait()
		except: pass
		self.demo.update_status(self.devname, "Status: Stopped")
	def stop(self):
		try: self.p.communicate("q")
		except: pass

class ffmpegThread(threading.Thread):
	def __init__(self, name, outname, outfmt, width, height, norm, input, demo):
		threading.Thread.__init__(self)
		self.devname = name
		self.outname = outname
		self.outfmt = outfmt
		self.width = width
		self.height = height
		self.norm = norm
		self.input = input
		self.action = "Capturing AVI"
		self.demo = demo
	def run(self):
		cmd = "ffmpeg -s %dx%d" % (self.width, self.height)
		if self.outfmt == V4L2_PIX_FMT_MJPEG or self.outfmt == V4L2_PIX_FMT_JPEG:
			cmd += " -f mjpeg -analyzeduration 0"
		elif self.outfmt == V4L2_PIX_FMT_H264:
			cmd += " -f h264 -analyzeduration 0"
		elif self.outfmt == V4L2_PIX_FMT_MP4V:
			cmd += " -f m4v -analyzeduration 0 "
		elif self.outfmt == V4L2_PIX_FMT_NV12:
			cmd += " -f video4linux2 -pix_fmt nv12"
		elif self.outfmt == V4L2_PIX_FMT_MP42:
			cmd += " -f mp4 -analyzeduration 0 "
			
		if self.outfmt == V4L2_PIX_FMT_UYVY:
			cmd += " -f video4linux2 -pix_fmt uyvy422"
		elif self.outfmt == V4L2_PIX_FMT_YUYV:
			cmd += " -f video4linux2 -pix_fmt yuyv422"
		elif self.outfmt == V4L2_PIX_FMT_GREY:
			cmd += " -f video4linux2 -pix_fmt gray"
		if self.norm == "NTSC":
			cmd += " -r 29.97"
		elif self.norm == "PAL":
			cmd += " -r 25"

		cmd += " -i " + self.devname + " -vcodec copy -acodec copy -y " + self.outname
		dprint(cmd)
		status = "Status: Stopped"
		try: self.p = subprocess.Popen(cmd, stdin=subprocess.PIPE, shell=True)
		except: status = "Could not open ffmpeg"
		try: self.p.wait()
		except: pass
		self.demo.update_status(self.devname, status)
	def stop(self):
		try: self.p.communicate("q")
		except: pass


class OldCaptureThread(threading.Thread):
	def __init__(self, name, outname, outfmt, width, height, norm, input, oneshot, demo):
		threading.Thread.__init__(self)
		self.devname = name
		self.outname = outname
		self.outfmt = outfmt
		self.width = width
		self.height = height
		self.norm = norm
		self.input = input
		self.action = "Capturing"
		self.quit = False
		self.oneshot = oneshot
		self.demo = demo
	def run(self):
		self.do_capture()
		if self.oneshot:
			self.demo.update_status(self.devname, "Status: Saved snapshot " + self.outname)
		else:
			self.demo.update_status(self.devname, "Status: Stopped")
	
	def do_capture(self):
		try: vd = os.open(self.devname, os.O_RDWR)
		except:
			print "Device "+self.devname+" not available or does not support multiple open"
			return
		try: 
			outf = open(self.outname, "w")
		except:
			print "Unable to open output file"
			return
			
		format = v4l2_format()
		format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
		format.fmt.pix.pixelformat = self.outfmt
		format.fmt.pix.width = self.width
		format.fmt.pix.height = self.height
		format.fmt.pix.bytesperline = 0
		format.fmt.pix.field = V4L2_FIELD_ANY
		try: ioctl(vd, VIDIOC_S_FMT, format)
		except:
			print "VIDIOC_S_FMT failed"
			vd.close()
			return
			
		reqbufs = v4l2_requestbuffers()
		reqbufs.count = 4
		reqbufs.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
		reqbufs.memory = V4L2_MEMORY_MMAP
		try: ioctl(vd, VIDIOC_REQBUFS, reqbufs)
		except IOError:
			print "VIDIOC_REQBUFS failed"
			vd.close()
			return
		lengths = []
		mmaps = []
		for i in range(0, reqbufs.count - 1):
			buf = v4l2_buffer()
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
			buf.memory = V4L2_MEMORY_MMAP
			buf.index = i
			try: ioctl(vd, VIDIOC_QUERYBUF, buf)
			except IOError: print "VIDIOC_QUERYBUF failed"
			lengths.append(buf.length)
			start = mmap(vd, buf.length, offset=buf.m.offset)
			mmaps.append(start)
			try: ioctl(vd, VIDIOC_QBUF, buf)
			except IOError: print "VIDIOC_QBUF failed"

		type = ctypes.c_int(V4L2_BUF_TYPE_VIDEO_CAPTURE)
		try: ioctl(vd, VIDIOC_STREAMON, type)
		except IOError: print "VIDIOC_STREAMON failed"
		dprint("Capturing started")
		
		poll = select.poll()
		poll.register(vd, select.POLLIN)
		while not self.quit:
			if poll.poll(5000) == []:
				print "Poll timed out"
				break
			buf = v4l2_buffer()
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
			buf.memory = V4L2_MEMORY_MMAP
			try: ioctl(vd, VIDIOC_DQBUF, buf)
			except IOError: 
				print "VIDIOC_DQBUF error"
				break
			mmaps[buf.index].seek(0)
			outf.write(mmaps[buf.index].read(buf.bytesused))
			try: ioctl(vd, VIDIOC_QBUF, buf)
			except IOError: print "VIDIOC_QBUF failed"
			if self.oneshot:
				break
		type = ctypes.c_int(V4L2_BUF_TYPE_VIDEO_CAPTURE)
		try: ioctl(vd, VIDIOC_STREAMOFF, type)
		except IOError: print "VIDIOC_STREAMOFF failed"
		os.close(vd)
		outf.close()
		dprint("Capturing stopped")
		
	def stop(self):
		self.quit = True


class CaptureThread(threading.Thread):
	def __init__(self, name, outname, outfmt, videnc, width, height, norm, input, oneshot, demo, ip = None, port = None):
		threading.Thread.__init__(self)
		self.devname = name
		self.outname = outname
		self.outfmt = outfmt
		self.videnc = videnc
		self.width = width
		self.height = height
		self.norm = norm
		self.input = input
		self.action = 'Capturing'
		self.quit = False
		self.oneshot = oneshot
		self.demo = demo
		self.sock = None
		if ip:
			self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
			self.sock.connect((ip, int(port)))
			self.action = 'Streaming'
			self.outname = '-'
			
	def run(self):
		cmd = [sys.path[0] + "/capture"]
		cmd.append('-d')
		cmd.append(self.devname)
		cmd.append('-o')
		cmd.append(self.outname)
		cmd.append('-f')
		if self.oneshot:
			cmd.append('1')
		else:
			cmd.append('0')
		cmd.append('-s')
		cmd.append('%dx%d' % (self.width, self.height))
		if self.norm == "PAL":
			cmd.append('-p')

		if self.outfmt == V4L2_PIX_FMT_NV12:
			cmd.append('-n')
		elif self.outfmt == V4L2_PIX_FMT_YUYV:
			cmd.append('-y')
		elif self.outfmt == V4L2_PIX_FMT_UYVY:
			cmd.append('-u')
		elif self.outfmt == V4L2_PIX_FMT_GREY:
			cmd.append('-8')
		elif self.outfmt == V4L2_PIX_FMT_MP4V:
			cmd.append('-4')
		elif self.outfmt == V4L2_PIX_FMT_H264:
			cmd.append('-x')
		elif self.outfmt == V4L2_PIX_FMT_MP2V:
			cmd.append('-2')
		elif self.outfmt == V4L2_PIX_FMT_MJPEG or self.outfmt == V4L2_PIX_FMT_JPEG:
			cmd.append('-J')
		elif self.outfmt == V4L2_PIX_FMT_MP42:
			if self.sock:
				cmd.append('-z')
			else:
				cmd.append('-m')
		elif self.outfmt == V4L2_PIX_FMT_MPEG:
			cmd.append('-t')
		if self.outfmt == V4L2_PIX_FMT_MP42 or self.outfmt == V4L2_PIX_FMT_MPEG:
			if self.videnc == V4L2_MPEG_VIDEO_ENCODING_MPEG_2:
				cmd.append('-2')
			elif self.videnc == V4L2_MPEG_VIDEO_ENCODING_MPEG_4_AVC:
				cmd.append('-x')
			elif self.videnc == V4L2_MPEG_VIDEO_ENCODING_MPEG_4:
				cmd.append('-4')
		dprint(' '.join(cmd))
		print ' '.join(cmd)

		status = "Status: Stopped"
		if self.sock:
			pr, pw = os.pipe()
			poll = select.poll()
			poll.register(pr, select.POLLIN)
			try:
				self.process = subprocess.Popen(cmd, stdout=pw)
			except:
				self.demo.update_status(self.devname, 
					"Could not open capture program - check sdk installation")
				return
			while self.process.returncode == None:
				self.process.poll()
				if poll.poll(100) == []:
					continue
				data = os.read(pr, 1400)
				try:
					self.sock.send(data)
				except:
					print "Socket send failed"
					status = "Status: Socket send failed"
					self.process.terminate()
					break
			try: self.process.wait()
			except: pass
			os.close(pr)
			os.close(pw)
			self.sock.close()
		else: 
			try:
				self.process = subprocess.Popen(cmd)
			except:
				self.demo.update_status(self.devname, 
					"Could not open capture program - check sdk installation")
				return
			try: self.process.wait()
			except: pass
		if self.process.returncode > 0:
			status = "Status: Capture Error"
		self.demo.update_status(self.devname, status)
	def stop(self):
		try: self.process.terminate()
		except: pass
		
		

class PlaybackThread(threading.Thread):
	def __init__(self, name, norm, filename, demo):
		threading.Thread.__init__(self)
		self.devname = name
		self.filename = filename
		self.norm = norm
		self.action = "Playing"
		self.quit = False
		self.demo = demo
	def run(self):
		cmd = [sys.path[0] + '/playback']
		cmd.append('-d')
		cmd.append(self.devname)
		if self.norm == "PAL":
			cmd.append('-p')
		cmd.append(self.filename)
		dprint(' '.join(cmd))
		status = "Status: Playback error"
		try:
			self.process = subprocess.Popen(cmd)
			try: self.process.wait()
			except: pass
			if self.process.returncode <= 0:
				status = "Status: Stopped"
		except: status = "Could not open playback program - check sdk installation"
		self.demo.update_status(self.devname, status)
	def stop(self):
		try: self.process.terminate()
		except: pass


class Demo:
	def delete_event(self, widget, event, data=None):
		return False

	def destroy(self, widget, data=None):
		gtk.main_quit()
		if self.vd != None:
			self.vd.close()

	def set_dev(self, widget, data):
		if self.vd != None:
			self.vd.close()
		try: self.vd = open(data, "rw")
		except: self.vd = None
		
		# clear the window and reenumerate the widgets
		self.window.remove(self.window.get_child())
		self.window.add(self.setup())

	def set_dim(self, widget, data):
		self.width = data[0]
		self.height = data[1]
		#dprint("set_dim", self.width, self.height)
		format = v4l2_format()
		format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
		try: ioctl(self.vd, VIDIOC_G_FMT, format)
		except IOError: return
		format.fmt.pix.width = self.width
		format.fmt.pix.height = self.height
		format.fmt.pix.bytesperline = 0
		dprint("new dim is", self.width, self.height)
		try: 
			ioctl(self.vd, VIDIOC_S_FMT, format)
		except IOError:
			print "Unable to set format"

	def set_format(self, widget, data=None):
		format = v4l2_format()
		format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
		try: ioctl(self.vd, VIDIOC_G_FMT, format)
		except IOError: return
		format.fmt.pix.pixelformat = data
		format.fmt.pix.field = V4L2_FIELD_ANY
		format.fmt.pix.bytesperline = 0
		dprint("new format is", data)
		try: 
			ioctl(self.vd, VIDIOC_S_FMT, format)
			self.outfmt = data
		except IOError:
			print "Unable to set format"
		# clear the window and reenumerate the widgets
		self.window.remove(self.window.get_child())
		self.window.add(self.setup())

	def set_input(self, widget, data=None):
		dprint("new input is", data)
		input = ctypes.c_int(data)
		self.input = int(data)
		try: ioctl(self.vd, VIDIOC_S_INPUT, input)
		except IOError: return

	def set_output(self, widget, data=None):
		dprint("new output is", data)
		output = ctypes.c_int(data)
		self.output = int(data)
		try: ioctl(self.vd, VIDIOC_S_OUTPUT, output)
		except IOError: return

	def set_audio_input(self, widget, data=None):
		dprint("new audio input is", data)
		audio = v4l2_audio()
		try: ioctl(self.vd, VIDIOC_G_AUDIO, audio)
		except IOError: return
		audio.index = data
		try: ioctl(self.vd, VIDIOC_S_AUDIO, audio)
		except IOError: return

	def set_audio_mode(self, widget, data=None):
		dprint("new audio mode is", data)
		audio = v4l2_audio()
		try: ioctl(self.vd, VIDIOC_G_AUDIO, audio)
		except IOError: return
		audio.mode = data
		try: ioctl(self.vd, VIDIOC_S_AUDIO, audio)
		except IOError: return

	def set_audio_output(self, widget, data=None):
		dprint("new audio input is", data)
		audio = v4l2_audioout()
		audio.index = data
		try: ioctl(self.vd, VIDIOC_S_AUDOUT, audio)
		except IOError: return

	def set_frame_interval(self, widget, index):
		frmivalenum = v4l2_frmivalenum()
		frmivalenum.index = index
		frmivalenum.pixel_format = self.outfmt
		frmivalenum.width = self.width
		frmivalenum.height = self.height
		try: ioctl(self.vd, VIDIOC_ENUM_FRAMEINTERVALS, frmivalenum)
		except IOError: return
		dprint("new frame interval index is", index, 
			frmivalenum.discrete.numerator,
			frmivalenum.discrete.denominator)
		cp = v4l2_capability()
		try: ioctl(self.vd, VIDIOC_QUERYCAP, cp)
		except IOError: return
		streamparm = v4l2_streamparm()
		if cp.capabilities & V4L2_CAP_VIDEO_CAPTURE:
			streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
			streamparm.parm.capture.timeperframe.numerator = frmivalenum.discrete.numerator
			streamparm.parm.capture.timeperframe.denominator = frmivalenum.discrete.denominator
		elif cp.capabilities & V4L2_CAP_VIDEO_OUTPUT:
			streamparm.type = V4L2_BUF_TYPE_VIDEO_OUTPUT
			streamparm.parm.output.timeperframe.numerator = frmivalenum.discrete.numerator
			streamparm.parm.output.timeperframe.denominator = frmivalenum.discrete.denominator
		try:
			ioctl(self.vd, VIDIOC_S_PARM, streamparm)
		except IOError: return

	def set_std(self, widget, data=None):
		dprint("new standard is", data)
		std = v4l2_std_id(data)
		if (std.value & V4L2_STD_NTSC):
			self.norm = "NTSC"
		elif (std.value & V4L2_STD_PAL):
			self.norm = "PAL"
		try: ioctl(self.vd, VIDIOC_S_STD, std)
		except IOError: print "unable to set standard"
		self.window.remove(self.window.get_child())
		self.window.add(self.setup())
	
	def set_ctrl(self, widget, id):
		value = int(widget.value)
		if V4L2_CTRL_ID2CLASS(id) == V4L2_CTRL_CLASS_USER:
			ctrl = v4l2_control()
			ctrl.id = id
			ctrl.value = value
			dprint("new ctrl", id, "is", ctrl.value)
			try: ioctl(self.vd, VIDIOC_S_CTRL, ctrl)
			except IOError: print "unable to set control"
		else:
			ctrl = v4l2_ext_control()
			ctrl.id = id
			ctrl.value = value
			ctrls = v4l2_ext_controls()
			ctrls.ctrl_class = V4L2_CTRL_ID2CLASS(id)
			ctrls.count = 1
			ctrls.controls = ctypes.pointer(ctrl)
			dprint("new ext ctrl", id, "is", ctrl.value)
			try: ioctl(self.vd, VIDIOC_S_EXT_CTRLS, ctrls)
			except IOError: print "unable to set ext control"
		return ctrl.value

	def get_ctrl(self, cid):
		ctrl = v4l2_control()
		ctrl.id = cid
		try: ioctl(self.vd, VIDIOC_G_CTRL, ctrl)
		except IOError: print "unable to get control"
		return ctrl.value

	def set_str_ctrl(self, widget, id, entry=None):
		if entry == None:
			entry = widget
		ctrl = v4l2_ext_control()
		ctrl.id = id
		ctrl.reserved = ctypes.cast(ctypes.c_char_p(entry.get_text()), ctypes.c_void_p)
		ctrls = v4l2_ext_controls()
		ctrls.ctrl_class = V4L2_CTRL_ID2CLASS(id)
		ctrls.count = 1
		ctrls.controls = ctypes.pointer(ctrl)
		dprint("new ext ctrl", id, "is", ctrl.reserved)

		try: ioctl(self.vd, VIDIOC_S_EXT_CTRLS, ctrls)
		except IOError: print "unable to set ext control"

	def set_opt(self, widget, opt, value):
		dprint("set_history", value)
		opt.set_history(value)
		opt.get_menu().get_active().activate()
	
	def set_toggle(self, widget, id):
		widget.value = widget.get_active()
		self.set_ctrl(widget, id)
		
	def set_audmode(self, widget, id):
		audio = v4l2_audio()
		audio.index = 0
		try: ioctl(self.vd, VIDIOC_G_AUDIO, audio)
		except IOError: return
		if widget.get_active():
			audio.mode |= V4L2_AUDMODE_AVL
		else:
			audio.mode &= ~V4L2_AUDMODE_AVL
		try: ioctl(self.vd, VIDIOC_S_AUDIO, audio)
		except IOError: print "Unable to set audio mode"


	def set_jpegcomp(self, widget, adj):
		jpegcomp = v4l2_jpegcompression()
		jpegcomp.quality = int(widget.value)
		dprint("new jpeg quality is", jpegcomp.quality)
		try: ioctl(self.vd, VIDIOC_S_JPEGCOMP, jpegcomp)
		except IOError:
			print "unable to set jpeg quality"
			try: 
				ioctl(self.vd, VIDIOC_G_JPEGCOMP, jpegcomp)
				adj.value = jpegcomp.quality
			except IOError:
				print "unable to get jpeg quality"
		
	def message_box(self, msg):
		msg = gtk.MessageDialog(self.window,
			gtk.DIALOG_MODAL,
			gtk.MESSAGE_INFO,
			gtk.BUTTONS_CLOSE,
			msg)
		msg.show()
		msg.run()
		msg.destroy()
		
	def message_box_yes_no(self, msg):
		msg = gtk.MessageDialog(self.window,
			gtk.DIALOG_MODAL,
			gtk.MESSAGE_INFO,
			gtk.BUTTONS_YES_NO,
			msg)
		msg.show()
		result = msg.run()
		msg.destroy()
		return result == gtk.RESPONSE_YES
		
	
	def is_device_busy(self):
		for thread in self.threads:
			if thread.devname == self.vd.name:
				if thread.is_alive():
					self.message_box("The device is busy. ("+thread.action+")")
					return True
				self.threads.remove(thread)
				break
		return False
	
	def take_snapshot(self, widget):
		if self.is_device_busy():
			return

		# Generate a unique filename
		i = 0
		while True:
			name = "out_%d.jpg" % i
			try: os.stat(name)
			except: break
			i += 1
		thread = CaptureThread(self.vd.name, name, V4L2_PIX_FMT_JPEG, -1, self.width, self.height, self.norm, self.input, True, self)
		self.threads.append(thread)
		thread.start()

	def launch_mplayer(self, widget):
		if self.is_device_busy():
			return
		if self.outfmt == V4L2_PIX_FMT_MP4V \
		or self.outfmt == V4L2_PIX_FMT_MP2V \
		or self.outfmt == V4L2_PIX_FMT_H264 \
		or self.outfmt == V4L2_PIX_FMT_MP42 \
		or self.outfmt == V4L2_PIX_FMT_MPEG:
			if not self.message_box_yes_no("The format %s is not recommended for preview. Preview anyway?" % (self.outfmt_name)):
				return
		thread = MplayerThread(self.vd.name, self.outfmt, self.width, self.height, self.norm, self.input, self)
		self.threads.append(thread)
		thread.start()
		self.status.set_text("Status: Previewing")
		return

	def get_videnc(self):
		ctrl = v4l2_ext_control()
		ctrl.id = V4L2_CID_MPEG_VIDEO_ENCODING
		ctrls = v4l2_ext_controls()
		ctrls.ctrl_class = V4L2_CTRL_ID2CLASS(ctrl.id)
		ctrls.count = 1
		ctrls.controls = ctypes.pointer(ctrl)
		try: 
			ioctl(self.vd, VIDIOC_G_EXT_CTRLS, ctrls)
		except IOError: print "unable to get ext control"
		return ctrl.value

	def launch_capture(self, widget):
		if self.is_device_busy():
			return
		name = "output"
		videnc = self.get_videnc()
		if self.outfmt == V4L2_PIX_FMT_NV12:
			name += ".nv12"
		elif self.outfmt == V4L2_PIX_FMT_YUYV:
			name += ".yuyv"
		elif self.outfmt == V4L2_PIX_FMT_UYVY:
			name += ".uyvy"
		elif self.outfmt == V4L2_PIX_FMT_GREY:
			name += ".grey"
		elif self.outfmt == V4L2_PIX_FMT_MP4V:
			name += ".m4v"
		elif self.outfmt == V4L2_PIX_FMT_H264:
			name += ".h264"
		elif self.outfmt == V4L2_PIX_FMT_MPEG:
			name += ".ts"
		elif self.outfmt == V4L2_PIX_FMT_MP2V:
			name += ".m2v"
		elif self.outfmt == V4L2_PIX_FMT_MJPEG or self.outfmt == V4L2_PIX_FMT_JPEG:
			name += ".mjpg"
		elif self.outfmt == V4L2_PIX_FMT_MP42:
			name += ".mp4"
		else:
			name += ".dat"
		save = gtk.FileChooserDialog("Save Capture As...", self.window, gtk.FILE_CHOOSER_ACTION_SAVE, (gtk.STOCK_CANCEL, gtk.RESPONSE_REJECT, gtk.STOCK_SAVE, gtk.RESPONSE_ACCEPT))
		save.set_default_response(1)
		save.set_do_overwrite_confirmation(True)
		save.set_current_name(name)
		if self.default_folder:
			save.set_current_folder(self.default_folder)
		save.show()
		if save.run() != gtk.RESPONSE_ACCEPT:
			save.destroy()
			return
		self.default_folder = save.get_current_folder()
		thread = CaptureThread(self.vd.name, save.get_filename(), self.outfmt, videnc, self.width, self.height, self.norm, self.input, False, self)
		save.destroy()
		self.threads.append(thread)
		thread.start()
		self.status.set_text("Status: Capturing")
		return

	def launch_stream(self, widget):
		if self.is_device_busy():
			return
		videnc = self.get_videnc()
		dialog = gtk.Dialog("UDP Stream...", self.window, gtk.DIALOG_MODAL | gtk.DIALOG_DESTROY_WITH_PARENT, (gtk.STOCK_CANCEL, gtk.RESPONSE_REJECT, gtk.STOCK_OK, gtk.RESPONSE_ACCEPT))
		label = gtk.Label("Address")
		label.show()
		dialog.vbox.pack_start(label)
		ip = gtk.Entry()
		if self.default_stream_ip:
			ip.set_text(self.default_stream_ip)
		else:
			ip.set_text('127.0.0.1')
		ip.show()
		dialog.vbox.pack_start(ip)
		label = gtk.Label("UDP Port")
		label.show()
		dialog.vbox.pack_start(label)
		port = gtk.Entry()
		if self.default_stream_port:
			port.set_text(self.default_stream_port)
		else:
			port.set_text('1234')
		port.show()
		dialog.vbox.pack_start(port)
		dialog.show()
		if dialog.run() != gtk.RESPONSE_ACCEPT:
			dialog.destroy()
			return
		self.default_stream_ip = ip.get_text()
		self.default_stream_port = port.get_text()
		thread = CaptureThread(self.vd.name, '-', self.outfmt, videnc, self.width, self.height, self.norm, self.input, False, self, ip.get_text(), port.get_text())
		dialog.destroy()
		self.threads.append(thread)
		self.status.set_text("Status: Streaming")
		thread.start()
		return

	def launch_capture_avi(self, widget):
		if self.is_device_busy():
			return
		name = "output"
		if self.outfmt == V4L2_PIX_FMT_NV12:
			name += "_nv12.avi"
		elif self.outfmt == V4L2_PIX_FMT_YUYV:
			name += "_yuyv.avi"
		elif self.outfmt == V4L2_PIX_FMT_UYVY:
			name += "_uyvy.avi"
		elif self.outfmt == V4L2_PIX_FMT_GREY:
			name += "_grey.avi"
		elif self.outfmt == V4L2_PIX_FMT_MP4V:
			name += "_mpeg4.avi"
		elif self.outfmt == V4L2_PIX_FMT_MP2V:
			name += "_mpeg2.avi"
		elif self.outfmt == V4L2_PIX_FMT_H264:
			name += "_h264.avi"
		elif self.outfmt == V4L2_PIX_FMT_MJPEG or self.outfmt == V4L2_PIX_FMT_JPEG:
			name += "_mjpeg.avi"
		elif self.outfmt == V4L2_PIX_FMT_MP42:
			name += ".avi"
			if not self.message_box_yes_no("The MP4 fragmented format is not recommended for capturing to AVI. Capture anyway?"):
				return
		else:
			name += ".avi"
		save = gtk.FileChooserDialog("Save Capture As...", self.window, gtk.FILE_CHOOSER_ACTION_SAVE, ("Cancel", 0, "Save", 1))
		save.set_default_response(1)
		save.set_do_overwrite_confirmation(True)
		save.set_current_name(name)
		save.show()
		if save.run() != 1:
			save.destroy()
			return
		thread = ffmpegThread(self.vd.name, save.get_filename(), self.outfmt, self.width, self.height, self.norm, self.input, self)
		save.destroy()
		self.threads.append(thread)
		thread.start()
		self.status.set_text("Status: Capturing")
		return

	def launch_playback(self, widget):
		if self.is_device_busy():
			return
		load = gtk.FileChooserDialog("Choose a file to playback:", self.window, gtk.FILE_CHOOSER_ACTION_OPEN, ("Cancel", 0, "Open", 1))
		filter = gtk.FileFilter()
		filter.add_pattern("*.h264")
		filter.add_pattern("*.264")
		filter.add_pattern("*.m4v")
		filter.add_pattern("*.m2v")
		filter.add_pattern("*.mpeg4")
		filter.add_pattern("*.mp4")
		filter.add_pattern("*.mp4f")
		filter.add_pattern("*.ts")
		#filter.add_pattern("*.mjpg")
		#filter.add_pattern("*.mjpeg")
		load.set_filter(filter)
		if self.default_folder:
			load.set_current_folder(self.default_folder)
		load.show()
		if load.run() != 1:
			load.destroy()
			return
		self.default_folder = load.get_current_folder()
		thread = PlaybackThread(self.vd.name, self.norm, load.get_filename(), self)
		load.destroy()
		self.threads.append(thread)
		thread.start()
		self.status.set_text("Status: Playing")
		return
		

	def stop_thread(self, widget):
		for thread in self.threads:
			if thread.devname == self.vd.name:
				if thread.is_alive():
					thread.stop()
				else:
					self.threads.remove(thread)
				break
		self.status.set_text("Status: Stopped")

	def stop_all_threads(self):
		for thread in self.threads:
			if (thread.is_alive()):
				thread.stop()
	def get_status_string(self):
		for thread in self.threads:
			if thread.devname == self.vd.name:
				if (thread.is_alive()):
					return "Status: "+thread.action
		return "Status: Stopped"
	def update_status(self, name, msg):
		if name == self.vd.name:
			self.status.set_text(msg)
		
	def __init__(self):
		self.threads = []
		self.window = gtk.Window(gtk.WINDOW_TOPLEVEL)
		self.window.connect("delete_event", self.delete_event)
		self.window.connect("destroy", self.destroy)
		self.window.set_border_width(10)
		self.window.set_title("V4L2 Capture Demo")
		self.default_stream_ip = None
		self.default_stream_port = None
		self.default_folder = os.getcwd()

		# open the device
		self.vd = None
		matches = glob.glob("/dev/video[0-9]*")
		if matches:
			matches.sort()
		for dev in matches:
			try:
				self.vd = open(dev, "rw")
				break
			except:
				continue
		
		self.window.add(self.setup())
		
		self.window.show()

		
	def setup(self):
		vbox = gtk.VBox(False, 10)
		
		
		# get the driver name
		
		hbox = gtk.HBox(False, 10)
		label = gtk.Label("Device Name:")
		hbox.pack_start(label, False, False, 0)
		label.show()

		opt = gtk.OptionMenu()
		menu = gtk.Menu()
		menudefault = 0
		i = 0
		matches = glob.glob("/dev/video[0-9]*")
		if matches:
			matches.sort()
		for dev in matches:
			menu.append(make_menu_item(dev, self.set_dev, dev))
			if self.vd != None and dev == self.vd.name:
				menudefault = i
			i += 1
		opt.set_menu(menu)
		opt.set_history(menudefault)
		hbox.pack_start(opt, False, False, 0)
		opt.show()
		vbox.pack_start(hbox, False, False, 0)
		hbox.show()

		quitbox = gtk.HBox(False, 10)

		if self.vd == None:
			hbox = gtk.HBox(False, 10)
			label = gtk.Label("Device failed to open, please try again")
			hbox.pack_start(label, False, False, 0)
			label.show()
			label.set_alignment(0,0)
			vbox.pack_start(hbox, False, False, 0)
			hbox.show()
			
			vbox.show()
		else:
			# get the driver name

			hbox = gtk.HBox(False, 10)
			label = gtk.Label("Driver Name:")
			hbox.pack_start(label, False, False, 0)
			label.show()

			cp = v4l2_capability()
			ioctl(self.vd, VIDIOC_QUERYCAP, cp)

			label = gtk.Label(cp.driver)
			hbox.pack_start(label, False, False, 0)
			label.show()

			label = gtk.Label("Version:")
			hbox.pack_start(label, False, False, 0)
			label.show()
			label = gtk.Label("%d.%d.%d" % (
				(cp.version >> 16) & 0xff, 
				(cp.version >> 8) & 0xff,
				cp.version & 0xff))
			hbox.pack_start(label, False, False, 0)
			label.show()
			vbox.pack_start(hbox, False, False, 0)
			hbox.show()

			hbox = gtk.HBox(False, 10)
			label = gtk.Label("Card Name:")
			hbox.pack_start(label, False, False, 0)
			label.show()

			label = gtk.Label(cp.card)
			hbox.pack_start(label, False, False, 0)
			label.show()
			vbox.pack_start(hbox, False, False, 0)
			hbox.show()

			hbox = gtk.HBox(False, 10)
			label = gtk.Label("Bus Info:")
			hbox.pack_start(label, False, False, 0)
			label.show()

			label = gtk.Label(cp.bus_info)
			hbox.pack_start(label, False, False, 0)
			label.show()
			vbox.pack_start(hbox, False, False, 0)
			hbox.show()


			if cp.capabilities & V4L2_CAP_VIDEO_CAPTURE:
				# enumerate video inputs
				hbox = gtk.HBox(False, 10)

				label = gtk.Label("Capture Input:")
				hbox.pack_start(label, False, False, 0)
				label.show()

				input = v4l2_input()
				input.index = 0
				opt = gtk.OptionMenu()
				menu = gtk.Menu()

				while 1:
					try: ioctl(self.vd, VIDIOC_ENUMINPUT, input)
					except IOError: break
					dprint("enumerated input: ", input.index, input.name)
					menu.append(make_menu_item(input.name, self.set_input, input.index))
					input.index += 1

				opt.set_menu(menu)
				hbox.pack_start(opt, False, False, 0)

				curinput = ctypes.c_int()
				try: ioctl(self.vd, VIDIOC_G_INPUT, curinput)
				except: pass
				self.input = curinput.value

				opt.set_history(self.input)
				opt.show()

				vbox.pack_start(hbox, False, False, 0)
				hbox.show()
			elif cp.capabilities & V4L2_CAP_VIDEO_OUTPUT:
				#enumerate video outputs
				hbox = gtk.HBox(False, 10)

				label = gtk.Label("Video Output:")
				hbox.pack_start(label, False, False, 0)
				label.show()

				output = v4l2_output()
				output.index = 0
				opt = gtk.OptionMenu()
				menu = gtk.Menu()

				while 1:
					try: ioctl(self.vd, VIDIOC_ENUMOUTPUT, output)
					except IOError: break
					dprint("enumerated output: ", output.index, output.name)
					menu.append(make_menu_item(output.name, self.set_output, output.index))
					output.index += 1

				opt.set_menu(menu)
				hbox.pack_start(opt, False, False, 0)

				curoutput = ctypes.c_int()
				try: ioctl(self.vd, VIDIOC_G_OUTPUT, curoutput)
				except: pass
				self.output = curoutput.value

				opt.set_history(self.output)
				opt.show()

				vbox.pack_start(hbox, False, False, 0)
				hbox.show()

			# create a option menu for video frame sizes

			if cp.capabilities & V4L2_CAP_VIDEO_CAPTURE:
				format = v4l2_format()
				format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
				try: ioctl(self.vd, VIDIOC_G_FMT, format)
				except: pass

				dprint("current format is 0x%08x" % format.fmt.pix.pixelformat)
				self.outfmt = format.fmt.pix.pixelformat
				self.width = format.fmt.pix.width
				self.height = format.fmt.pix.height
				dprint("current size is ", self.width, self.height)

				hbox = gtk.HBox(False, 10)
				label = gtk.Label("Size:")
				hbox.pack_start(label, False, False, 0)
				label.show()

				opt = gtk.OptionMenu()
				menu = gtk.Menu()
				menudefault = 0
				i = 0
				for dim in [
					[720,576, "D1 PAL"],
					[704,576, "D1 PAL "],
					[720,480, "D1 NTSC "],
					[704,480, "D1 NTSC"],
					[640,480, "VGA"],
					[352,288, "CIF"],
					[320,240, "SIF"],
					[176,144, "QCIF"],
					[160,128, "QSIF"],
					]:
					menu.append(make_menu_item("%dx%d %s" % (dim[0],dim[1],dim[2]), self.set_dim, dim[0:2]))
					if self.width == dim[0] and self.height == dim[1]:
						menudefault = i
					i += 1
				opt.set_menu(menu)
				opt.set_history(menudefault)
				hbox.pack_start(opt, False, False, 0)
				opt.show()

				vbox.pack_start(hbox, False, False, 0)
				hbox.show()

			# enumerate video standards

			hbox = gtk.HBox(False, 10)

			label = gtk.Label("Video Standard:")
			hbox.pack_start(label, False, False, 0)
			label.show()

			std = v4l2_standard()
			std.index = 0

			curstd = v4l2_std_id()
			try: ioctl(self.vd, VIDIOC_G_STD, curstd)
			except: pass
			dprint("current std is", curstd.value)
			if (curstd.value & V4L2_STD_NTSC):
				self.norm = "NTSC"
			elif (curstd.value & V4L2_STD_PAL):
				self.norm = "PAL"
			else:
				self.norm = None

			if self.norm != None:
				opt = gtk.OptionMenu()
				menu = gtk.Menu()
				menudefault = 0
				menuindex = 0

				while 1:
					try: ioctl(self.vd, VIDIOC_ENUMSTD, std)
					except IOError: break
					dprint("enumerated std: ", std.index, std.name, std.id)
					if std.name == "NTSC" or std.name == "PAL":
						if std.id == curstd.value: menudefault = menuindex
						menu.append(make_menu_item(std.name, self.set_std, std.id))
						menuindex += 1
					std.index += 1

				opt.set_menu(menu)
				hbox.pack_start(opt, False, False, 0)
				opt.set_history(menudefault)
				opt.show()

				vbox.pack_start(hbox, False, False, 0)
				hbox.show()


			# enumerate the capture formats

			if cp.capabilities & V4L2_CAP_VIDEO_CAPTURE:
				hbox = gtk.HBox(False, 10)

				label = gtk.Label("Format:")
				hbox.pack_start(label, False, False, 0)
				label.show()


				fmtdesc = v4l2_fmtdesc()
				fmtdesc.index = 0
				fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE

				opt = gtk.OptionMenu()
				menu = gtk.Menu()
				menudefault = 0
				jpegseen = False
				menuindex = 0
				while 1:
					try: ioctl(self.vd, VIDIOC_ENUM_FMT, fmtdesc)
					except IOError: break
					dprint("enumerated format:", fmtdesc.index, fmtdesc.pixelformat, fmtdesc.description)
					if fmtdesc.pixelformat == format.fmt.pix.pixelformat:
						menudefault = menuindex
						self.outfmt_name = fmtdesc.description
					fmtdesc.index += 1
					if fmtdesc.pixelformat == V4L2_PIX_FMT_JPEG or fmtdesc.pixelformat == V4L2_PIX_FMT_MJPEG:
						if jpegseen:
							continue
						jpegseen = True
					menu.append(make_menu_item(fmtdesc.description, self.set_format, fmtdesc.pixelformat))
					menuindex += 1
				opt.set_menu(menu)
				hbox.pack_start(opt, False, False, 0)
				opt.set_history(menudefault)
				opt.show()

				vbox.pack_start(hbox, False, False, 0)
				hbox.show()

			# enumerate controls

			notebook = gtk.Notebook()
			
			cbox = gtk.VBox(False, 0)

			queryctrl = v4l2_queryctrl()
			ctrl_id = 0
			use_next_ctrl = True

			while True:
				if use_next_ctrl:
					ctrl_id |= V4L2_CTRL_FLAG_NEXT_CTRL
				else:
					ctrl_id += 1
				queryctrl.id = ctrl_id
				try:
					ioctl(self.vd, VIDIOC_QUERYCTRL, queryctrl)
					if use_next_ctrl:
						ctrl_id = queryctrl.id
				except IOError:
					if ctrl_id == V4L2_CTRL_FLAG_NEXT_CTRL:
						# next controls are not supported
						use_next_ctrl = False
						ctrl_id = V4L2_CID_BASE

						cbox = gtk.VBox(False, 0)
						notebook.append_page(cbox, gtk.Label("Controls"))
						cbox.show()
						cbox.set_spacing(-15)
						continue
					elif use_next_ctrl:
						break
					elif ctrl_id >= V4L2_CID_BASE and ctrl_id < V4L2_CID_LASTP1:
						continue
					elif ctrl_id == V4L2_CID_LASTP1:
						ctrl_id = V4L2_CID_PRIVATE_BASE
						continue
					break


				if queryctrl.type == V4L2_CTRL_TYPE_CTRL_CLASS:
					cbox = gtk.VBox(False, 0)
					notebook.append_page(cbox, gtk.Label(queryctrl.name))
					cbox.show()
					cbox.set_spacing(-15)
					

					#hbox = gtk.HBox(False, 10)
					#label = gtk.Label(queryctrl.name + ":")
					#label.set_markup("<u>" + queryctrl.name + "</u>:")
					#hbox.pack_start(label, False, False, 0)
					#label.show()
					#cbox.pack_start(hbox, False, False, 0)
					#hbox.show()
					continue
				if queryctrl.flags & V4L2_CTRL_FLAG_DISABLED:
					continue

				dprint("enumerated control:", queryctrl.name, queryctrl.type)
				hbox = gtk.HBox(False, 0)
				hbox.set_spacing(-10)
				label = gtk.Label(queryctrl.name + ":")
				hbox.pack_start(label, False, False, 10)
				label.show()

				# Read the default value

 				if queryctrl.type != V4L2_CTRL_TYPE_STRING:
	 				ctrl = v4l2_control()
					ctrl.id = queryctrl.id
					ctrl.value = queryctrl.default
					try: ioctl(self.vd, VIDIOC_G_CTRL, ctrl)
					except: print "unable to get control"
					default_value = ctrl.value
				else:
					ctrl = v4l2_ext_control()
					ctrl.id = queryctrl.id
					ctrl.string = ctypes.cast(ctypes.create_string_buffer(queryctrl.maximum), ctypes.c_char_p)
					ctrls = v4l2_ext_controls()
					ctrls.ctrl_class = V4L2_CTRL_ID2CLASS(ctrl.id)
					ctrls.count = 1
					ctrls.controls = ctypes.pointer(ctrl)
					try:
						ioctl(self.vd, VIDIOC_G_EXT_CTRLS, ctrl, True)
					except: print "unable to get ext control"
					default_value = ctrl.string
					if default_value == None:
						default_value = ""

				if queryctrl.type == V4L2_CTRL_TYPE_BOOLEAN:
					#opt = gtk.OptionMenu()
					#menu = gtk.Menu()
					#menu.append(make_menu_item("Disabled", self.set_ctrl, queryctrl.id, 0))
					#menu.append(make_menu_item("Enabled", self.set_ctrl, queryctrl.id, 1))
					#opt.set_menu(menu)
					#opt.set_history(default_value)
					#hbox.pack_start(opt, False, False, 0)
					#opt.show()
					chk = gtk.CheckButton("Enabled")
					chk.set_active(default_value)
					hbox.pack_start(chk, False, False, 10);
					chk.show()
					
					space = gtk.Label("")
					hbox.pack_start(space, True, True, 0)
					space.show()
					if queryctrl.flags & V4L2_CTRL_FLAG_READ_ONLY:
						chk.set_sensitive(False)
						button = gtk.Button("Read")
						button.connect("clicked",
							lambda button, chk, cid:
								chk.set_active(self.get_ctrl(cid)),
							chk, queryctrl.id)
					else:
						chk.connect("toggled", self.set_toggle, queryctrl.id)
						button = gtk.Button("Default")
						button.connect("clicked",
							lambda button, chk, active:
								chk.set_active(active),
							chk, queryctrl.default)
					hbox.pack_start(button, False, False, 10)
					button.show()

				elif queryctrl.type == V4L2_CTRL_TYPE_MENU:
					opt = gtk.OptionMenu()
					menu = gtk.Menu()
					querymenu = v4l2_querymenu()
					querymenu.id = queryctrl.id
					querymenu.index = queryctrl.minimum
					menucurrent = 0
					menudefault = 0
					while querymenu.index <= queryctrl.maximum:
						try: ioctl(self.vd, VIDIOC_QUERYMENU, querymenu)
						except: break
						dprint("menu item:", querymenu.name)
						if querymenu.name != "":
							menu.append(make_menu_item(querymenu.name, self.set_ctrl, queryctrl.id, querymenu.index))
							if querymenu.index < default_value:
								menucurrent += 1
							if querymenu.index < queryctrl.default:
								menudefault += 1
						querymenu.index += 1
					opt.set_menu(menu)
					opt.set_history(menucurrent)
					hbox.pack_start(opt, False, False, 10)
					opt.show()
					space = gtk.Label("")
					hbox.pack_start(space, True, True, 10)
					space.show()

					if queryctrl.flags & V4L2_CTRL_FLAG_READ_ONLY:
						querymenu.set_sensitive(False)
					else:
						button = gtk.Button("Default")
						hbox.pack_start(button, False, False, 10)
						button.connect("clicked", self.set_opt, opt, menudefault)
						button.show()
						
				elif queryctrl.flags & V4L2_CTRL_FLAG_READ_ONLY:
					label = gtk.Label(default_value)
					hbox.pack_start(label, False, False, 10)
					label.show()

				elif queryctrl.type == V4L2_CTRL_TYPE_STRING:
					entry = gtk.Entry(queryctrl.maximum)
					entry.set_text(default_value)
					hbox.pack_start(entry, True, True, 10)
					entry.connect("activate", self.set_str_ctrl, queryctrl.id)
					entry.show()

					button = gtk.Button("Set")
					hbox.pack_start(button, False, False, 10)
					button.connect("clicked", self.set_str_ctrl, queryctrl.id, entry)
					button.show()

				else:
					adj = gtk.Adjustment(default_value,
						queryctrl.minimum,
						queryctrl.maximum,
						queryctrl.step,
						queryctrl.step)
					adj.connect("value_changed", 
						lambda adj, id:
							adj.set_value(self.set_ctrl(adj, id)),
						queryctrl.id)
					hscale = gtk.HScale(adj)
					hscale.set_digits(0)
					hscale.set_value_pos(gtk.POS_LEFT)
					hscale.connect("format-value", format_k)
					hbox.pack_start(hscale, True, True, 10)
					hscale.show()

					button = gtk.Button("Default")
					hbox.pack_start(button, False, False, 10)
					button.connect("clicked", 
						lambda button, adj, value:
							adj.set_value(value),
						adj, queryctrl.default)
					button.show()
					
				cbox.pack_start(hbox, False, False, 10)
				hbox.show()

			# jpeg quality
			jpegcomp = v4l2_jpegcompression()
			try:
				ioctl(self.vd, VIDIOC_G_JPEGCOMP, jpegcomp)
			except:
				jpegcomp = None
			if jpegcomp != None:
				hbox = gtk.HBox(False, 10)
				label = gtk.Label("JPEG Quality:")
				hbox.pack_start(label, False, False, 10)
				label.show()
				adj = gtk.Adjustment(jpegcomp.quality, 1, 100, 1, 1)
				adj.connect("value_changed", self.set_jpegcomp, adj)
				hscale = gtk.HScale(adj)
				hscale.set_digits(0)
				hscale.set_value_pos(gtk.POS_LEFT)
				hbox.pack_start(hscale, True, True, 10)
				hscale.show()

				#vbox.pack_start(hbox, False, False, 0)
				jbox=gtk.VBox(False, 10)
				jbox.pack_start(hbox, False, False, 10)
				jbox.show()
				notebook.append_page(jbox, gtk.Label("JPEG Controls"))
				hbox.show()
				
				
			# frame intervals
			frmivalenum = v4l2_frmivalenum()
			frmivalenum.index = 0
			frmivalenum.pixel_format = self.outfmt
			frmivalenum.width = self.width
			frmivalenum.height = self.height
			streamparm = v4l2_streamparm()
			if cp.capabilities & V4L2_CAP_VIDEO_CAPTURE:
				streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
			elif cp.capabilities & V4L2_CAP_VIDEO_OUTPUT:
				streamparm.type = V4L2_BUF_TYPE_VIDEO_OUTPUT
			try:
				ioctl(self.vd, VIDIOC_ENUM_FRAMEINTERVALS, frmivalenum)
				ioctl(self.vd, VIDIOC_G_PARM, streamparm)
			except:
				frmivalenum = None
			if frmivalenum != None:
				hbox = gtk.HBox(False, 10)
				label = gtk.Label("Frame Rate:")
				hbox.pack_start(label, False, False, 0)
				label.show()
				
				opt = gtk.OptionMenu()
				menu = gtk.Menu()
				menudefault = 0
				if cp.capabilities & V4L2_CAP_VIDEO_CAPTURE:
					timeperframe = streamparm.parm.capture.timeperframe
				elif cp.capabilities & V4L2_CAP_VIDEO_OUTPUT:
					timeperframe = streamparm.parm.output.timeperframe
				while True:
					dprint("enumerated interval:", frmivalenum.index, frmivalenum.discrete.numerator, frmivalenum.discrete.denominator)
					menu.append(make_menu_item("%.2f fps"%(frmivalenum.discrete.denominator * 1.0 / frmivalenum.discrete.numerator), self.set_frame_interval, frmivalenum.index))
					if frmivalenum.discrete.numerator == timeperframe.numerator \
					and frmivalenum.discrete.denominator == timeperframe.denominator:
						menudefault = frmivalenum.index
					frmivalenum.index += 1
					try: ioctl(self.vd, VIDIOC_ENUM_FRAMEINTERVALS, frmivalenum)
					except IOError: break
				opt.set_menu(menu)
				hbox.pack_start(opt, False, False, 0)
				opt.set_history(menudefault)
				opt.show()

				vbox.pack_start(hbox, False, False, 0)
				hbox.show()
				
			# audio
			if cp.capabilities & V4L2_CAP_VIDEO_CAPTURE:
				audio = v4l2_audio()
				audio.index = 0
				try: ioctl(self.vd, VIDIOC_ENUMAUDIO, audio)
				except IOError: audio = None
				if audio != None:
					hbox = gtk.HBox(False, 10)
					label = gtk.Label("Audio Input:")
					hbox.pack_start(label, False, False, 0)
					label.show()
					has_avl = False;

					opt = gtk.OptionMenu()
					menu = gtk.Menu()
					while True:
						dprint("enumerated audio:", audio.index, audio.name)
						menu.append(make_menu_item(audio.name, self.set_audio_input, audio.index))
						audio.index += 1
						if audio.capability & V4L2_AUDCAP_AVL:
							has_avl = True
						try: ioctl(self.vd, VIDIOC_ENUMAUDIO, audio)
						except IOError: break
					opt.set_menu(menu)
					hbox.pack_start(opt, False, False, 0)
					try: ioctl(self.vd, VIDIOC_G_AUDIO, audio)
					except IOError: print "VIDIOC_G_AUDIO failed"
					opt.set_history(audio.index)
					opt.show()
					if has_avl:
						chk = gtk.CheckButton("Auto Volume Level")
						chk.set_active(audio.mode & V4L2_AUDMODE_AVL)
						chk.connect("toggled", self.set_audmode, chk)
						hbox.pack_start(chk, False, False, 0);
						chk.show()

					vbox.pack_start(hbox, False, False, 0)
					hbox.show()
			elif cp.capabilities & V4L2_CAP_VIDEO_OUTPUT:
				audio = v4l2_audioout()
				audio.index = 0
				try: ioctl(self.vd, VIDIOC_ENUMAUDOUT, audio)
				except IOError: audio = None
				if audio != None:
					hbox = gtk.HBox(False, 10)
					label = gtk.Label("Audio Output:")
					hbox.pack_start(label, False, False, 0)
					label.show()

					opt = gtk.OptionMenu()
					menu = gtk.Menu()
					while True:
						dprint("enumerated audio:", audio.index, audio.name)
						menu.append(make_menu_item(audio.name, self.set_audio_output, audio.index))
						audio.index += 1
						try: ioctl(self.vd, VIDIOC_ENUMAUDOUT, audio)
						except IOError: break
					opt.set_menu(menu)
					hbox.pack_start(opt, False, False, 0)
					try: ioctl(self.vd, VIDIOC_G_AUDOUT, audio)
					except IOError: print "VIDIOC_G_AUDOUT failed"
					opt.set_history(audio.index)
					opt.show()

					vbox.pack_start(hbox, False, False, 0)
					hbox.show()

			hrule = gtk.HSeparator()
			hrule.show()
			vbox.pack_end(hrule, False, False, 0)
			
			hbox = gtk.HBox(False, 20)
			hbox.pack_start(vbox, False, True, 0)
			vbox.show()

			#vrule = gtk.VSeparator()
			#vrule.show()
			#hbox.pack_start(vrule, False, False, 0)
			
			#hbox.pack_start(cbox, True, True, 0)
			#cbox.show()
			hbox.pack_start(notebook, True, True, 0)
			notebook.show()
			notebook.set_current_page(0)
		
			vbox = gtk.VBox(False, 10)
			vbox.pack_start(hbox, True, True, 0)
			hbox.show()
			
			if cp.capabilities & V4L2_CAP_VIDEO_CAPTURE:
				if jpegseen:
					button = gtk.Button("Snapshot")
					button.connect_object("clicked", self.take_snapshot, None)
					button.show()
					quitbox.pack_start(button, False, False, 0)

				mplayer = gtk.Button("Preview")
				mplayer.connect_object("clicked", self.launch_mplayer, None)
				mplayer.show()
				quitbox.pack_start(mplayer, False, False, 0)

				capture = gtk.Button("Save")
				capture.connect_object("clicked", self.launch_capture, None)
				capture.show()
				quitbox.pack_start(capture, False, False, 0)

				#capture = gtk.Button("Capture to AVI")
				#capture.connect_object("clicked", self.launch_capture_avi, None)
				#capture.show()
				#vbox.pack_start(capture, False, False, 0)
				stream = gtk.Button("Stream")
				stream.connect_object("clicked", self.launch_stream, None)
				stream.show()
				quitbox.pack_start(stream, False, False, 0)

				button = gtk.Button("Stop")
				button.connect_object("clicked", self.stop_thread, None)
				button.show()
				quitbox.pack_start(button, False, False, 0)

			elif cp.capabilities & V4L2_CAP_VIDEO_OUTPUT:
				
				playback = gtk.Button("Play")
				playback.connect_object("clicked", self.launch_playback, None)
				playback.show()
				quitbox.pack_start(playback, False, False, 0)

				button = gtk.Button("Stop")
				button.connect_object("clicked", self.stop_thread, None)
				button.show()
				quitbox.pack_start(button, False, False, 0)
			
		
		
		if self.vd != None:
			statusFrame = gtk.Frame()
			self.status = gtk.Label(self.get_status_string())
			statusFrame.add(self.status)
			statusFrame.show()
			self.status.set_alignment(0.0, 0.5)
			self.status.show()
			quitbox.pack_start(statusFrame, True, True, 0)
		else:
			button = gtk.Button("Refresh")
			button.connect_object("clicked", self.set_dev, button, "/dev/video0")
			button.show()
			quitbox.pack_start(button, False, False, 0)

		button = gtk.Button("Quit")
		button.connect_object("clicked", gtk.Widget.destroy, self.window)
		button.show()
		quitbox.pack_start(button, False, False, 0)
		
		vbox.pack_start(quitbox, False, True, 0)
		quitbox.show()
		
		vbox.show()
		return vbox

	def main(self):
		gtk.gdk.threads_init()
		gtk.main()
		self.stop_all_threads()

if __name__ == "__main__":
	demo = Demo()
	demo.main()
