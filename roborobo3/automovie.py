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

replim = 2 # Only keep 2 movies of a rep per experiment (faster movies)


def make_video(curdir, nbjobs=1):
    print("looking up files in ", curdir)
    rep_out = re.match(r".*rep(\d+).*", curdir)
    if rep_out:
        rep = int(rep_out[1])
    else:
        rep = 0
    if rep < replim:  # Only deal with the first reps
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
            verbose = sys.stdout.isatty()
            newmov.write_videofile(outname, fps=60, verbose=verbose, progress_bar=verbose, threads=nbjobs)
            print("{} created".format(basename(outname)))
    else:
        print("Do not make movie for this rep {}, already have others".format(rep))
    for pngfile in files:
        remove(pngfile)

def tolerant_make_video(curdir, nbjobs=1):
    try:
        make_video(curdir)
    except Exception as e:
        print("error with {}:\n{}, {}".format(curdir, type(e), e),
              file=sys.stderr)

### MAIN ###
def main():
    me = singleton.SingleInstance()
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
        delayed(tolerant_make_video)(curdir, 1)
            for curdir in irglob(screendirpattern)
                if glob(j(curdir, 'screenshot_custom_*_gen*.png')) # if there is screenshots to process
    )
    del me # del single instance mode

if __name__ == "__main__":
    main()
