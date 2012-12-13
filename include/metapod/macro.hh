// Copyright 2012,
//
// Sébastien Barthélémy (Aldebaran Robotics)
//
// This file is part of metapod.
// metapod is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// metapod is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// You should have received a copy of the GNU Lesser General Public License
// along with metapod.  If not, see <http://www.gnu.org/licenses/>.

#ifndef METAPOD_MACRO_CONFIG_HH
# define METAPOD_MACRO_CONFIG_HH

# if __GNUC__ >= 4 && __GNUC_MINOR__ >= 3
// hot is supported on functions since GCC 4.3.
// It is also supported on labels since GCC 4.8
#  define METAPOD_HOT __attribute__ ((hot))
# else
#  define METAPOD_HOT
#endif

#endif
