# Copyright (c) 2011, Lukas Jirkovsky
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the <organization> nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

FIND_PATH(OpenEXR_INCLUDE_DIR OpenEXR/ImfRgbaFile.h)

FIND_LIBRARY(OpenEXR_IlmImf_LIBRARY NAMES IlmImf)
FIND_LIBRARY(OpenEXR_Imath_LIBRARY NAMES Imath)
FIND_LIBRARY(OpenEXR_Half_LIBRARY NAMES Half)
FIND_LIBRARY(OpenEXR_Iex_LIBRARY NAMES Iex)
FIND_LIBRARY(OpenEXR_IlmThread_LIBRARY NAMES IlmThread)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(OpenEXR DEFAULT_MSG
    OpenEXR_Half_LIBRARY OpenEXR_Iex_LIBRARY OpenEXR_Imath_LIBRARY
    OpenEXR_IlmImf_LIBRARY OpenEXR_IlmThread_LIBRARY OpenEXR_INCLUDE_DIR
)

if(OpenEXR_FOUND)
    SET(OpenEXR_LIBRARIES ${OpenEXR_IlmImf_LIBRARY}
        ${OpenEXR_Imath_LIBRARY} ${OpenEXR_Half_LIBRARY}
        ${OpenEXR_Iex_LIBRARY} ${OpenEXR_IlmThread_LIBRARY} )

    # Add imported targets.
    if(NOT TARGET OpenEXR::OpenEXR)
        foreach(_OpenEXR_lib Half Iex Imath IlmImf IlmThread)
            add_library(OpenEXR::${_OpenEXR_lib} UNKNOWN IMPORTED)
            set_target_properties(OpenEXR::${_OpenEXR_lib} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${OpenEXR_INCLUDE_DIR}")
            set_property(TARGET OpenEXR::${_OpenEXR_lib} APPEND PROPERTY IMPORTED_LOCATION "${OpenEXR_${_OpenEXR_lib}_LIBRARY}")
        endforeach()
        add_library(OpenEXR::OpenEXR INTERFACE IMPORTED)
        set_target_properties(OpenEXR::OpenEXR PROPERTIES INTERFACE_LINK_LIBRARIES "OpenEXR::Half;OpenEXR::Iex;OpenEXR::Imath;OpenEXR::IlmImf;OpenEXR::IlmThread")
    endif()
endif()
