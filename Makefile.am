ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include -L/usr/local/lib

techlibs = -lm -lach -lc -lhubotech -ldaemonizer -lrt -lhuboparams

# List the binary files you want to generate:
bin_PROGRAMS = tech-sample tech-arm-sample tech-trajectory-sample


# For each one, list the .cpp sources:
tech_sample_SOURCES = src/tech-sample.cpp
# And then the libraries which they should link to:
tech_sample_LDADD = $(techlibs)

# Replace any dashes with underscores
tech_arm_sample_SOURCES = src/tech-arm-sample.cpp
tech_arm_sample_LDADD = $(techlibs)


tech_trajectory_sample_SOURCES = src/tech-trajectory-sample.cpp
tech_trajectory_sample_LDADD = $(techlibs)


