CC     := gcc
LD      := $(CC)

INC     := -Isrc -I/home/yathartha/MuJoCo/MuJoCo_Hopper-master/mjpro150/include
CFLAGS  := -fPIC -std=gnu11 -Wall -Wextra -Wpedantic -O3 -ffast-math -march=native -Wno-unused-function
LDFLAGS := -shared -L/home/yathartha/MuJoCo/MuJoCo_Hopper-master/mjpro150/bin -Wl,-rpath,/home/yathartha/MuJoCo/MuJoCo_Hopper-master/mjpro150/bin
LIBS    := -lmujoco150 -lglew -lGL -l:libglfw.so.3
OUT     := libslip.so

SRC     := $(wildcard src/*.c)
OBJ     := $(patsubst src/%.c,obj/%.o,$(SRC))

vpath %.c src

define make-goal
$1/%.o: %.c
	$(CC) $(CFLAGS) $(INC) -MMD -c $$< -o $$@
endef

all: checkdirs build

clean:
	rm -f $(OUT)
	rm -rf obj/
	rm -f test

test: all
	$(CC) main.c -std=c99 ./libslip.so -Isrc -Imjpro150/include -lm -Wl,-rpath,.,-rpath,./mjpro150/bin -o test

playback: all
	$(CC) playback.c -std=c99 ./libslip.so -Isrc -Imjpro150/include -lm -Wl,-rpath,.,-rpath,./mjpro150/bin -o playback

joystick: all
	$(CC) main_joy.c -std=c99 ./libslip.so -Isrc -Imjpro150/include -lm -Wl,-rpath,.,-rpath,./mjpro150/bin -o joystick

build: $(OBJ)
	$(LD) $^ -o $(OUT) $(LDFLAGS) $(LIBS)

checkdirs: obj

obj:
	@mkdir -p $@

$(foreach bdir,obj,$(eval $(call make-goal,$(bdir))))

.PHONY: all checkdirs clean

-include $(OBJ:%.o=%.d)
