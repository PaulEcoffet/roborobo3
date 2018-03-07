from moviepy.video.io.VideoFileClip import VideoFileClip
from moviepy.video.io.ImageSequenceClip import ImageSequenceClip
from moviepy.video.compositing.concatenate import concatenate_videoclips
import sys
if sys.version_info >= (3, 5):
    from glob import iglob, glob
    def rglob(*args, **kwargs):
        kwargs['recursive'] = True
        return glob(*args, **kwargs)
    def irglob(*args, **kwargs):
        kwargs['recursive'] = True
        return iglob(*args, **kwargs)
else:
    from glob2 import iglob, glob
    rglob = glob
    irglob = iglob

from os.path import join as j
from os.path import basename, dirname, exists
from os import remove
from collections import defaultdict
import re
from joblib import Parallel, delayed
import sys

# Ensure automovie is not already running.
from tendo import singleton
me = singleton.SingleInstance()


def make_video(curdir):
    print("looking up files in ", curdir)
    files = glob(j(curdir, 'screenshot_custom_*.png'))
    filesbygen = defaultdict(list)
    for fname in files:
        name = basename(fname)
        out = re.match(r"""screenshot_custom_.+_
                       gen_(?P<gen>\d+)
                       (?:_ind_(?P<ind>\d+))?
                       .*\.png""", name, re.VERBOSE)
        if out:
            gen = out.group("gen")
            if out.group("ind"):
                gen += "i" + out.group("ind")
            filesbygen[gen].append(fname)
    for gen, files in filesbygen.items():
        print("making movie for {}".format(gen))
        outname = j(curdir, 'mov_{}.mp4'.format(gen))
        try:
            newmov = ImageSequenceClip(sorted(files), fps=60)
        except:
            print(files)
            raise
        if exists(outname):
            print("{} already found, concatenating.".format(basename(outname)))
            prev = VideoFileClip(outname)
            newmov = concatenate_videoclips([prev, newmov])
        newmov.write_videofile(outname, fps=60)
        print("{} created".format(basename(outname)))
        for pngfile in files:
            remove(pngfile)

def tolerant_make_video(curdir):
    try:
        make_video(curdir)
    except Exception as e:
        print("error with {}:\n{}, {}".format(curdir, type(e), e),
              file=sys.stderr)

### MAIN ###

maindir = '/data/logs/'
try:
     maindir = sys.argv[1]
except Exception:
    pass
nbjobs = 1
try:
    nbjobs = int(sys.argv[2])
except Exception:
    pass

screendirpattern = j(maindir, '**/screenshots/')
Parallel(n_jobs=nbjobs)(
    delayed(tolerant_make_video)(curdir)
        for curdir in irglob(screendirpattern)
            if glob(j(curdir, 'screenshot_custom_*_gen*.png')) # if there is screenshots to process
)

del me # del single instance mode
