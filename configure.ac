
AC_INIT(m3ens, 0.1.0, info@mekabot.com)

AC_CONFIG_AUX_DIR(config)
AC_CONFIG_MACRO_DIR([m4])

AM_CONFIG_HEADER(config.h)

AM_INIT_AUTOMAKE

AC_PROG_CXX
AC_PROG_CC
AC_DISABLE_STATIC
AC_PROG_LIBTOOL

#------------------------------------------------------------------------------
# Python bindings
#------------------------------------------------------------------------------
AM_PATH_PYTHON(2.3)
AC_PROG_SWIG(1.3.21)
SWIG_ENABLE_CXX
SWIG_PYTHON

#------------------------------------------------------------------------------
# Virtual mode flag
#------------------------------------------------------------------------------
AC_ARG_ENABLE([virtual-only],
[  --enable-virtual-only    Turn off realtime and ethercat],
[case "${enableval}" in
  yes) rtai=false ;;
  no)  rtai=true ;;
  *) AC_MSG_ERROR([bad value ${enableval} for --enable-virtual-only]) ;;
esac],[rtai=true])
AM_CONDITIONAL([RTAI], [test x$rtai = xtrue])

#TODO: have autoconf find correct rtai dir
AC_SUBST(RTAI_DIR,[/usr/realtime-2.6.32.11-rtai-3.8.1])

#TODO: have autoconf find correct ros dir
AC_SUBST(ROS_DIR,[/opt/ros/cturtle/])

#------------------------------------------------------------------------------
# Eigen
#------------------------------------------------------------------------------
PKG_CHECK_MODULES([EIGEN2], [eigen2 >= 2.0.15])

#------------------------------------------------------------------------------
# Yaml-cpp
#------------------------------------------------------------------------------
PKG_CHECK_MODULES([YAMLCPP], [yaml-cpp >= 0.2.5])

#------------------------------------------------------------------------------
# Protobuf
#------------------------------------------------------------------------------
PKG_CHECK_MODULES([PROTOBUF], [protobuf >= 2.2.0a])

AC_OUTPUT( Makefile m3ens.pc python/Makefile src/m3simc/controllers/Makefile)




