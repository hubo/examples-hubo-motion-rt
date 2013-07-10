ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include -L/usr/local/lib

ctrllibs = -lm -lach -lc -lhuboctrl -ldaemonizer -lrt -lhuboparams -lncurses

# List the binary files you want to generate:
bin_PROGRAMS = ctrl-sample ctrl-arm-sample ctrl-trajectory-sample trajectory-follow


# For each one, list the .cpp sources:
ctrl_sample_SOURCES = src/ctrl-sample.cpp
# And then the libraries which they should link to:
ctrl_sample_LDADD = $(ctrllibs)

# Replace any dashes with underscores
ctrl_arm_sample_SOURCES = src/ctrl-arm-sample.cpp
ctrl_arm_sample_LDADD = $(ctrllibs)


ctrl_trajectory_sample_SOURCES = src/ctrl-trajectory-sample.cpp
ctrl_trajectory_sample_LDADD = $(ctrllibs)

trajectory_follow_SOURCES = src/trajectory-follow.cpp
trajectory_follow_LDADD = $(ctrllibs)


