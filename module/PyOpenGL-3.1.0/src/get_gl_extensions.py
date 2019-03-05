#! /usr/bin/env python
"""Script to download OpenGL extensions header and produce wrappers

This script is the mechanism that allows for automatically
wrapping new extensions with basic ctypes-level functionality.
It doesn't do everything, but it approximates the same level
of access as GLEW or pyglet will give you.

The script also downloads the extension specification into .txt
files that sit next to the same-named .py files, the purpose of
this is to allow developers to grep through the source files 
looking for common features, such as references to the glGet*
tables.

glGetBoolean/glGetInteger/glGetFloat/glGetDouble 
    
    A CSV file in this directory controls registration of 
    constants for use with glGet, the format of the file is
    a tab-delimited format with the constants in the first 
    column.  Records are as follows:
    
    For no registration (e.g. when you need a custom function):

        CONSTANT
    
    For a static size:
    
        CONSTANT	(1,)
    
    For dynamic dictionary-based lookup of size based on another
    parameter:
    
        CONSTANT	pname	LOOKUPCONSTANT	(1,)	LOOKUPCONSTANT	(4,)	...

    Note that only constants which appear in a downloadable 
    specification (.txt file) under a New Tokens header with a note
    that they can be passed to glGetBoolean will be so registered.
"""
import urllib, os, sys, re, string, traceback, logging, textwrap
from funcparse import Helper, Function
EXTENSION_HEADER_SOURCE = 'http://www.opengl.org/registry/api/glext.h'
MISSING_HEADER_SOURCE = 'http://glhpp.googlecode.com/svn-history/r2/trunk/glext_missing.h.in'
#ROOT_EXTENSION_SOURCE = 'http://oss.sgi.com/projects/ogl-sample/registry/'
ROOT_EXTENSION_SOURCE = 'http://www.opengl.org/registry/specs/'
AUTOGENERATION_SENTINEL = """### DO NOT EDIT above the line "END AUTOGENERATED SECTION" below!"""
AUTOGENERATION_SENTINEL_END = """### END AUTOGENERATED SECTION"""
if not os.path.isdir( os.path.join('..','OpenGL','GL') ):
    raise RuntimeError( """Only run this script with the src directory as the current working directory""" )

log = logging.getLogger( 'autogen' )

def download( url ):
    """Download the given url, informing the user of what we're doing"""
    sys.stderr.write( 'Download: %r\n'%(url,))
    file = urllib.urlopen( url )
    return file.read()


registry = {}

def nameToPath( name ):
    return os.path.join( * name.split( '_',2 ) )
def nameToPathMinusGL( name ):
    return "/".join( name.split( '_',2 )[1:] )

def indent( text, indent='\t' ):
    return "\n".join([
        '%s%s'%(indent,line) 
        for line in text.splitlines()
    ])
    
# Don't know how Tarn got the api_versions, short of manually entering them...
WRAPPER_TEMPLATE = """'''Autogenerated by get_gl_extensions script, do not edit!'''
from OpenGL import platform as _p, constants as _cs, arrays
from OpenGL.GL import glget
import ctypes
EXTENSION_NAME = %(constantModule)r
def _f( function ):
    return _p.createFunction( function,%(dll)s,%(constantModule)r,%(deprecatedFlag)r)
%(constants)s
%(declarations)s
%(deprecated)s
"""

WRAPPER_TEMPLATE_NO_FUNCTIONS = """'''Autogenerated by get_gl_extensions script, do not edit!'''
from OpenGL import platform as _p
from OpenGL.GL import glget
EXTENSION_NAME = %(constantModule)r
%(constants)s
%(deprecated)s
"""
WRAPPER_TEMPLATE_NOTHING = """'''Autogenerated by get_gl_extensions script, do not edit!'''
EXTENSION_NAME = %(constantModule)r
%(deprecated)s
"""

INIT_TEMPLATE = """
def glInit%(camelModule)s%(owner)s():
    '''Return boolean indicating whether this extension is available'''
    from OpenGL import extensions
    return extensions.hasGLExtension( EXTENSION_NAME )
"""
FINAL_MODULE_TEMPLATE = """'''OpenGL extension %(owner)s.%(module)s

This module customises the behaviour of the 
OpenGL.raw.%(prefix)s.%(owner)s.%(module)s to provide a more 
Python-friendly API

%(overview)sThe official definition of this extension is available here:
%(root)s%(owner)s/%(module)s.txt
'''
from OpenGL import platform, constant, arrays
from OpenGL import extensions, wrapper
from OpenGL.GL import glget
import ctypes
from OpenGL.raw.%(prefix)s.%(owner)s.%(module)s import *
"""

class Module( Helper ):
    root = ROOT_EXTENSION_SOURCE
    targetDirectory = os.path.join( '..','OpenGL')
    rawTargetDirectory = os.path.join( '..','OpenGL','raw')
    prefix = 'GL'
    dll = '_p.GL'
    defineFinder = re.compile( r'\#define[ \t]+([a-zA-Z0-9_]+)[ \t]*(0x[0-9a-fA-F]+)' )
    functionFinder = re.compile( r'GLAPI[ \t]+(.*?)[ \t]+APIENTRY[ \t]+([a-zA-Z0-9_]+)[ \t]*\(' )
    signatureFinderTemplate = r'typedef[ \t]+%(returnTypeRE)s[ \t]+\(APIENTRYP[ \t]+PFN%(nameUpper)sPROC\)[ \t]*(\(.*?\))[;]'
    typeDefFinder = re.compile( r'typedef[ \t]+(([a-zA-Z0-9_]+[ \t]*)+);' )
    
    @property
    def RAW_MODULE_TEMPLATE( self ):
        if self.functions:
            return WRAPPER_TEMPLATE + INIT_TEMPLATE
        elif self.constants:
            return WRAPPER_TEMPLATE_NO_FUNCTIONS + INIT_TEMPLATE
        else:
            return WRAPPER_TEMPLATE_NOTHING + INIT_TEMPLATE
    
    def __init__( self, name, segments, header ):
        log.info( 'name: %r', name )
        if not name.startswith( 'GL_' ):
            name = 'GL_'+name
        self.name = name
        self.segments = segments
        self.header = header
        try:
            self.prefix, self.owner, self.module = name.split('_',2)
            self.sentinelConstant = '%s_%s'%(self.owner,self.module)
            
        except ValueError:
            if name.endswith( 'SGIX' ):
                self.prefix = "GL"
                self.owner = 'SGIX'
                self.module = name[3:-4]
                self.sentinelConstant = '%s%s'%(self.module,self.owner)
            else:
                log.error( """Unable to parse module name: %s""", name )
                raise
        if self.module[0] in string.digits:
            self.module = 'GL_%s'%(self.module,)
        self.camelModule = "".join([x.title() for x in self.module.split('_')])
        self.rawModule = self.module
        
        # XXX need to figure this out better
        self.rawOwner = self.owner
        while self.owner and self.owner[0] in string.digits:
            self.owner = self.owner[1:]
        self.rawPathName = os.path.join( self.rawTargetDirectory, self.prefix, self.owner, self.module+'.py' )
        self.pathName = os.path.join( self.targetDirectory, self.prefix, self.owner, self.module+'.py' )
        
        self.findFunctions()
        self.constantModule = '%(prefix)s_%(owner)s_%(rawModule)s'%self
        if self.rawModule.endswith( '_DEPRECATED' ):
            self.constantModule = self.constantModule[:-len('_DEPRECATED')]
            self.deprecatedFlag = True 
        else:
            self.deprecatedFlag = False
        specification = self.getSpecification()
        self.overview = ''
        if self.header.includeOverviews:
            for title,section in specification.blocks( specification.source ):
                if title.startswith( 'Overview' ):
                    self.overview = 'Overview (from the spec)\n%s\n\n'%(
                        indent( section.replace('\xd4','O').replace('\xd5','O').decode( 'ascii', 'ignore' ).encode( 'ascii', 'ignore' ) )
                    )
                    break

    def shouldReplace( self ):
        """Should we replace the given filename?"""
        filename = self.pathName
        if not os.path.isfile(
            filename
        ):
            return True
        else:
            hasLines = 0
            for line in open( filename ):
                if line.strip() == AUTOGENERATION_SENTINEL_END.strip():
                    return True
                hasLines = 1
            if not hasLines:
                return True
        return False


    def findFunctions( self ):
        """Find all of our function definitions"""
        result = []
        for segment in self.segments:
            for match in self.functionFinder.finditer(segment):
                returnType, name = match.groups()
                nameUpper = re.escape(name.upper())
                returnTypeRE = re.escape( returnType )
                signatureFinder = re.compile( self.signatureFinderTemplate%locals() )
                try:
                    signature = signatureFinder.search( segment ).group(1)
                    result.append( Function( returnType, name, signature ))
                except AttributeError:
                    log.warn( 
                        "Couldn't find signature for function %s %s",
                        returnType,name,
                    )
        self.functions = result
    def declarations( self ):
        """
        DECLARE_VOID_EXT(glPointParameterfARB, (GLenum pname, GLfloat param), (pname, param))
        DECLARE_VOID_EXT(glPointParameterfvARB, (GLenum pname, const GLfloat* param), (pname, param))
        """
        result = []
        for function in self.functions:
            result.append( function.declaration() )
        return "\n".join( result )
    def functionNames( self ):
        """
        "glPointParameterfARB",
        "glPointParameterfvARB",
        """
        result = []
        for function in self.functions:
            result.append( '"%s",'%(function.name,))
        return "\n".join(result)
    def swigFunctionDeclarations( self ):
        """
        void glPointParameterfARB(GLenum pname, GLfloat param);
        DOC(glPointParameterfARB, "glPointParameterfARB(pname, param) -> None")

        void glPointParameterfvARB(GLenum pname, const GLfloat* param);
        DOC(glPointParameterfvARB, "glPointParameterfvARB(pname, param) -> None")
        """
        result = []
        for segment in self.segments:
            for match in self.typeDefFinder.finditer( segment ):
                result.append( match.group(0))
        for function in self.functions:
            result.append( '%(returnType)s %(name)s%(signature)s;'%function )
            result.append( 'DOC(%(name)s, "%(name)s%(pysignature)s")'%function )
        return "\n".join( result )
    GLGET_CONSTANT = """glget.addGLGetConstant( %(name)s, %(size)s )"""
    def constants( self ):
        """Retrieve constants from the segments

        This is, of course, all heuristically done :)
        """
        result = []
        glget_set = []
        glGets = self.getSpecification().glGetConstants()
        glGetSizes = self.header.glGetSizes
        for segment in self.segments:
            for match in self.defineFinder.finditer( segment ):
                name,value = match.groups()
                value = int(value,0)
                result.append( '%(name)s 0x%(value)X'%locals() )
                if name in glGets or name in glGetSizes:
                    size = glGetSizes.get( name, [] )
                    if len(size) == 0: # not yet specified...
                        glGetSizes[ name ] = []
                    elif len(size) == 1: # static size...
                        size = size[0]
                        glget_set.append(
                            self.GLGET_CONSTANT %locals()
                        )
                    else:
                        # param name, then (key,value) for rest of elements
                        param = size[0]
                        rest = size[1:]
                        set = {}
                        while rest:
                            current = rest[:2]
                            del rest[:2]
                            if len(current) == 2:
                                set[current[0]] = current[1]
                            else:
                                log.warn( 
                                    """Incorrect format for glGet constant %s (unevent set of values)""",
                                    name,
                                )
                        size = '{ %s }'%(
                            ','.join([
                                '%s : %s'%(
                                    key,value 
                                )
                                for (key,value) in set 
                            ])
                        )
                        glget_set.append(
                            """glget.addGLGetConstant( %(name)s, %(size)s, %(param)r )"""%locals()
                        )
        if result:
            constants = "\n".join( result )
            result = [
                '_p.unpack_constants( """%(constants)s""", globals())'%locals()
            ]
        if glget_set:
            result.extend( glget_set )
        return "\n".join( result )
    SPEC_EXCEPTIONS = {
        # different URLs... grr...
        '3DFX/multisample': 'http://oss.sgi.com/projects/ogl-sample/registry/3DFX/3dfx_multisample.txt',
        #'EXT/color_matrix': 'http://oss.sgi.com/projects/ogl-sample/registry/SGI/color_matrix.txt',
        #'EXT/texture_cube_map': 'http://oss.sgi.com/projects/ogl-sample/registry/ARB/texture_cube_map.txt',
        'SGIS/fog_function': 'http://oss.sgi.com/projects/ogl-sample/registry/SGIS/fog_func.txt',
    }
    def getSpecification( self ):
        """Retrieve our specification document...
        
        Retrieves the .txt file which defines this specification,
        allowing us to review the document locally in order to provide
        a reasonable wrapping of it...
        """
        specFile = os.path.splitext( self.pathName )[0] + '.txt'
        specURLFragment = nameToPathMinusGL(self.name)
        if specURLFragment in self.SPEC_EXCEPTIONS:
            specURL = self.SPEC_EXCEPTIONS[ specURLFragment ]
        else:
            specURL = '%s/%s.txt'%( 
                self.root, 
                specURLFragment,
            )
        if not os.path.isfile( specFile ):
            try:
                data = download(specURL)
            except Exception, err:
                log.warn( """Failure downloading specification %s: %s""", specURL, err )
                data = ""
            else:
                try:
                    open(specFile,'w').write( data )
                except IOError, err:
                    pass
        else:
            data = open( specFile ).read()
        if 'Error 404' in data:
            log.info( """Spec 404: %s""", specURL)
            data = ''
        return Specification( data )
    def process( self ):
        """(re)Wrap the given module"""
        # first the raw wrapped API...
        directory = os.path.dirname(self.rawPathName)
        try:
            os.makedirs( directory )
        except os.error:
            pass
        if not os.path.isfile( os.path.join(directory, '__init__.py')):
            open( os.path.join(directory, '__init__.py'),'w').write( 
                '''"""OpenGL Extensions"""'''
            )
        current = ''
        toWrite = self.RAW_MODULE_TEMPLATE % self
        try:
            current = open( self.rawPathName, 'r').read()
        except Exception, err:
            pass 
        if current.strip() != toWrite.strip():
            fh = open( self.rawPathName, 'w')
            fh.write( toWrite )
            fh.close()
        if self.shouldReplace( ):
            # now the final module with any included custom code...
            toWrite = FINAL_MODULE_TEMPLATE % self
            current = ''
            try:
                current = open( self.pathName, 'r').read()
            except Exception, err:
                pass 
            else:
                found = current.rfind( '\n'+AUTOGENERATION_SENTINEL_END )
                if found >= -1:
                    if current[:found].strip() == toWrite.strip():
                        # we aren't going to change anything...
                        return False
                    found += len( '\n' + AUTOGENERATION_SENTINEL_END )
                    current = current[found:]
                else:
                    current = ''
            try:
                fh = open( self.pathName, 'w')
            except IOError, err:
                log.warn( "Unable to create module for %r %s", self.name, err )
                return False
            else:
                fh.write( toWrite )
                fh.write( AUTOGENERATION_SENTINEL_END )
                fh.write( current )
                fh.close()
                return True
        return False
    
    def deprecated( self ):
        """Produce import line for deprecated functions if appropriate"""
        name = self.name + '_DEPRECATED'
        if self.header.registry.get( name ):
            return '''# import deprecated
from OpenGL.raw.%(prefix)s.%(owner)s.%(module)s_DEPRECATED import *'''%self
        return ''

class VersionModule( Module ):
    """Module representing an OpenGL version's extension to the spec"""
    targetDirectory = os.path.join( '..','OpenGL')
    rawTargetDirectory = os.path.join( '..','OpenGL','raw')
    prefix = 'GL'
    RAW_MODULE_TEMPLATE = WRAPPER_TEMPLATE
    def getSpecification( self ):
        """Retrieve our specification document...
        
        Retrieves the .txt file which defines this specification,
        allowing us to review the document locally in order to provide
        a reasonable wrapping of it...
        """
        return Specification( '' )

class Specification( object ):
    """Parser for parsing OpenGL specifications for interesting information
    
    """
    def __init__( self, source ):
        """Store the source text for the specification"""
        self.source = source
    def blocks( self, data ):
        """Retrieve the set of all blocks"""
        data = data.splitlines()
        title = []
        block = []
        for line in data:
            if line and line.lstrip() == line:
                if block:
                    yield "\n".join(title), textwrap.dedent( "\n".join(block) )
                    title = [ ]
                    block = [ ]
                title.append( line )
            else:
                block.append( line )
        if block:
            yield "\n".join(title), textwrap.dedent( "\n".join(block) )
    def constantBlocks( self ):
        """Retrieve the set of constant blocks"""
        for title,block in self.blocks( self.source ):
            if title and title.startswith( 'New Tokens' ):
                yield block
    def glGetConstants( self ):
        """Retrieve the set of constants which pass to glGet* functions"""
        table = {}
        for block in self.constantBlocks():
            for title, section in self.blocks( block ):
                for possible in (
                    'GetBooleanv','GetIntegerv','<pname> of Get'
                ):
                    if possible in title:
                        for line in section.splitlines():
                            line = line.strip().split()
                            if len(line) == 2:
                                constant,value = line 
                                table['GL_%s'%(constant,)] = value 
                        break
        return table

class Header( object ):
    """Manages the overall header source
    
    registry -- registry of extensions/versions found with the 
        header segments that define them...
    includeOverviews -- if True, include the specification's 
        overviews in the indivdual extensions
    """
    registry = None
    includeOverviews = True
    def getFile( self ):
        """Load or download the source of the glext.h header"""
        if not os.path.isfile( 'glext.h' ):
            data = download( EXTENSION_HEADER_SOURCE )
            open( 'glext.h', 'w').write( data )
        else:
            data = open( 'glext.h' ).read()
        if not os.path.isfile( 'glext_missing.h.in' ):
            missing = download( MISSING_HEADER_SOURCE )
            open( 'glext_missing.h.in', 'w' ).write( missing )
        else:
            missing = open( 'glext_missing.h.in' ).read()
        data += '\n'
        data += missing 
        return data
    def getRegistry( self ):
        """Retrieve a parsed registry of extensions/versions
        
        This uses accidents of the header definition to produce
        the results, but the header is fairly consistent...
        
        returns { name: segments} to pass to Module init
        """
        if self.registry:
            return self.registry
        file = self.getFile()
        index = file.find( '#define GL_GLEXT_VERSION' )
        file = file[index:]
        extensions = file.split( '\n#ifndef ' )[1:]
        for item in extensions:
            name, definition = item.split( None, 1 )
            definition = '#ifndef '+item
            registry.setdefault( name, []).append( definition )
        self.registry = registry
        return registry
    def iterModules( self ):
        """Yield each Module( name, segments ) for all extensions
        
        extensions do *not* include the GL core versions...
        """
        items = self.getRegistry().items()
        items.sort()
        for name, segments in items:
            if name in ('APIENTRY','APIENTRYP','GLAPI'):
                continue
            if not name.startswith( 'GL_VERSION' ):
                yield Module( name, segments, header=self )
            else:
                yield VersionModule( name, segments, header=self )
    def iterVersions( self ):
        """Yield each Version( name, segments ) for all versions"""
        items = self.getRegistry().items()
        items.sort()
        for name, segments in items:
            if name.startswith( 'GL_VERSION' ):
                yield Version( name, segments )

    def autoGenerate( self ):
        """Autogenerate all Modules in this header"""
        new = {}
        total = count = 0
        for module in self.iterModules():
            if module.process( ):
                new[module.constantModule] = module
                count += 1
            total += 1
        return total, count
    
    def constantSections( self ):
        """Print the constant sections for all modules"""
        for module in self.iterModules():
            module.getSpecification()
            for constant, value in module.getSpecification().glGetConstants().items():
                #print title
                print constant
    
    glGetSizes = {}
    def loadGLGetSizes( self ):
        """Load manually-generated table of glGet* sizes"""
        table = self.glGetSizes
        try:
            lines = [
                line.split('\t')
                for line in open( 'glgetsizes.csv' ).read().splitlines()
            ]
        except IOError, err:
            pass 
        else:
            for line in lines:
                if line and line[0]:
                    table[line[0].strip('"')] = [
                        v for v in [
                            v.strip('"') for v in line[1:]
                        ]
                        if v
                    ]
    def saveGLGetSizes( self ):
        """Save out sorted list of glGet sizes to disk"""
        items = self.glGetSizes.items()
        items.sort()
        data = "\n".join([
            '%s\t%s'%(
                key,"\t".join(value)
            )
            for (key,value) in items 
        ])
        open( 'glgetsizes.csv','w').write( data )

if __name__ == "__main__":
    logging.basicConfig()
    log.setLevel( logging.WARN )
    header = Header()
    header.loadGLGetSizes()
    total,count = Header().autoGenerate()
    print '%s total %s replaced'%(total,count)
    header.saveGLGetSizes()
    #header.constantSections()
