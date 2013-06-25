# M3 -- Meka Robotics Robot Components
# Copyright (c) 2010 Meka Robotics
# Author: edsinger@mekabot.com (Aaron Edsinger)

# 
# M3 is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# M3 is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General Public License
# along with M3.  If not, see <http://www.gnu.org/licenses/>.

ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -g 

#SUBDIRS =	src/m3simc/controllers \
#		python

SUBDIRS =	python


dist_bin_SCRIPTS =

python_install:
	$(MAKE) -C python install

proto:
	$(MAKE) -C src/m3simc/controllers proto


proto_install:
	$(MAKE) -C src/m3simc/controllers proto_install

components:
	$(MAKE) -C src/m3simc/controllers

components_install:
	$(MAKE) -C src/m3simc/controllers install



#pkgconfigdir = $(libdir)/pkgconfig
#pkgconfig_DATA = m3ens.pc

EXTRA_DIST = \
	autogen.sh \
	configure


