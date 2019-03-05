'''Autogenerated by xml_generate script, do not edit!'''
from OpenGL import platform as _p, arrays
# Code generation uses this
from OpenGL.raw.GL import _types as _cs
# End users want this...
from OpenGL.raw.GL._types import *
from OpenGL.raw.GL import _errors
from OpenGL.constant import Constant as _C

import ctypes
_EXTENSION_NAME = 'GL_NV_bindless_texture'
def _f( function ):
    return _p.createFunction( function,_p.PLATFORM.GL,'GL_NV_bindless_texture',error_checker=_errors._error_checker)

@_f
@_p.types(_cs.GLuint64,_cs.GLuint,_cs.GLint,_cs.GLboolean,_cs.GLint,_cs.GLenum)
def glGetImageHandleNV(texture,level,layered,layer,format):pass
@_f
@_p.types(_cs.GLuint64,_cs.GLuint)
def glGetTextureHandleNV(texture):pass
@_f
@_p.types(_cs.GLuint64,_cs.GLuint,_cs.GLuint)
def glGetTextureSamplerHandleNV(texture,sampler):pass
@_f
@_p.types(_cs.GLboolean,_cs.GLuint64)
def glIsImageHandleResidentNV(handle):pass
@_f
@_p.types(_cs.GLboolean,_cs.GLuint64)
def glIsTextureHandleResidentNV(handle):pass
@_f
@_p.types(None,_cs.GLuint64)
def glMakeImageHandleNonResidentNV(handle):pass
@_f
@_p.types(None,_cs.GLuint64,_cs.GLenum)
def glMakeImageHandleResidentNV(handle,access):pass
@_f
@_p.types(None,_cs.GLuint64)
def glMakeTextureHandleNonResidentNV(handle):pass
@_f
@_p.types(None,_cs.GLuint64)
def glMakeTextureHandleResidentNV(handle):pass
@_f
@_p.types(None,_cs.GLuint,_cs.GLint,_cs.GLuint64)
def glProgramUniformHandleui64NV(program,location,value):pass
@_f
@_p.types(None,_cs.GLuint,_cs.GLint,_cs.GLsizei,arrays.GLuint64Array)
def glProgramUniformHandleui64vNV(program,location,count,values):pass
@_f
@_p.types(None,_cs.GLint,_cs.GLuint64)
def glUniformHandleui64NV(location,value):pass
@_f
@_p.types(None,_cs.GLint,_cs.GLsizei,arrays.GLuint64Array)
def glUniformHandleui64vNV(location,count,value):pass
