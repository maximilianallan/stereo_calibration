# DO NOT DELETE THIS LINE -- make depend depends on it.


# Edit the lines below to point to any needed include and link paths
# Or to change the compiler's optimization flags
CC = g++
MY_CUSTOM_INCLUDE_PATH = ./libcvd ./gvars4 ./TooN `pkg-config --cflags opencv`
MY_CUSTON_LINK_PATH = ./libcvd ./gvars5 ./TooN -lblas --llapack `pkg-config --libs opencv` 
COMPILEFLAGS = -I MY_CUSTOM_INCLUDE_PATH -D_LINUX -D_REENTRANT -Wall -g -O3 -march=nocona -msse3 
LINKFLAGS = -L MY_CUSTOM_LINK_PATH -lGVars3 -lcvd -lopencv_highgui -lopencv_calib3d

# Edit this line to change video source
VIDEOSOURCE = VideoSource_Linux_OpenCV.o

#OBJECTS=	main.o\
		GLWindow2.o\
		GLWindowMenu.o\
		$(VIDEOSOURCE)\
		System.o \
		ATANCamera.o\
		KeyFrame.o\
		MapPoint.o\
		Map.o\
		SmallBlurryImage.o\
		ShiTomasi.o \
		HomographyInit.o \
		MapMaker.o \
		Bundle.o \
		PatchFinder.o\
		Relocaliser.o\
		MiniPatch.o\
		MapViewer.o\
		ARDriver.o\
		EyeGame.o\
		Tracker.o\
		modifiedOpenCV.o\
		ATANRectifiedCamera.o

CALIB_OBJECTS=	GLWindow2.o\
		GLWindowMenu.o\
		$(VIDEOSOURCE)\
		CalibImage.o \
		CalibCornerPatch.o\
		ATANCamera.o \
		CameraCalibrator.o\
		HomographyInit.o\
		modifiedOpenCV.o

All: CameraCalibrator

#PTAM: $(OBJECTS)
#	$(CC) -o PTAM $(OBJECTS) $(LINKFLAGS)

CameraCalibrator:$(CALIB_OBJECTS)
	$(CC) -o CameraCalibrator $(CALIB_OBJECTS) $(LINKFLAGS)


%.o: %.cc
	$(CC) $< -o $@ -c $(COMPILEFLAGS)

clean:
	rm *.o


depend:
	rm dependecies; touch dependencies
	makedepend -fdependencies $(INCLUDEFLAGS) $(MOREINCS) *.cc *.h


-include dependencies









