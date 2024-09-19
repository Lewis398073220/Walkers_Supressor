
cur_dir := $(dir $(lastword $(MAKEFILE_LIST)))

obj_s := $(patsubst $(cur_dir)%,%,$(wildcard $(cur_dir)*.S))
obj_c := $(patsubst $(cur_dir)%,%,$(wildcard $(cur_dir)*.c))
obj_cpp := $(patsubst $(cur_dir)%,%,$(wildcard $(cur_dir)*.cpp))

obj-y := $(obj_c:.c=.o) $(obj_s:.S=.o) $(obj_cpp:.cpp=.o)

subdir-ccflags-y += \
    -Iservices/bt_app \
    -Iservices/bone_sensor \
    $(BLUETOOTH_ADAPTER_INCLUDES) \
    -Iservices/resources \
    -Iutils/cqueue \
    -Iapps/anc/inc

ifneq ($(SPEECH_ALGO_DSP),)
CFLAGS_audio_test_speech_algo_dsp.o += -DSPEECH_ALGO_DSP
ccflags-y += -Iservices/speech_algo_dsp \
             -Iutils/kfifo
endif

ifeq ($(AUDIO_DYNAMIC_BOOST),1)
CFLAGS_audio_test_cmd.o += -D__AUDIO_DYNAMIC_BOOST__
endif

ifeq ($(A2DP_KARAOKE),1)
CFLAGS_audio_test_cmd.o += -DA2DP_KARAOKE
endif