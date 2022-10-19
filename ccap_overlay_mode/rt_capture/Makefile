.SUFFIXES : .x .o .c .s

#CC := aarch64-linux-gnu-gcc  
#STRIP := aarch64-linux-gnu-strip

TARGET0 = rt_display_demo
SRCS0 := rt_display.c
TARGET1 = rt_overlay_demo
SRCS1 := overlay2display.c
TARGET2 = capure2display
SRCS2 := capure2display.c
LIBS= -lgcc -lpthread

all: 
	$(CC) $(SRCS0) -o $(TARGET0)
	$(CC) $(SRCS1) -o $(TARGET1)
	$(CC) $(SRCS2) -o $(TARGET2)
	$(STRIP) $(TARGET0) 
	$(STRIP) $(TARGET1)
	$(STRIP) $(TARGET2)
clean:
	rm -f *.o 
	rm -f *.x 
	rm -f *.flat
	rm -f *.map
	rm -f temp
	rm -f *.img
	rm -f $(TARGET0)
	rm -f $(TARGET1)
	rm -f $(TARGET2)
	rm -f *.gdb
	rm -f *.bak
