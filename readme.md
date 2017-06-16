# FPGA Replay Examples

This repository contains a handful of examples I've created
for the FPGA Replay board using the Replay Framework.

  * example_audio - Play audio from an SD card via the FPGA.
  * example_audio_visualiser - Adds a L/R waveform visualisation.

# Building

In order to build the examples you'll need to copy the replay\_lib/
and lib/ directories from the [Replay Library SVN](http://svn.fpgaarcade.com/)
into the repo root, or (preferably) provide sym links to those
directories in your Replay library checkout.

Should any examples fail to build with the latest version of the
Replay Library please let me know. 

The examples have been tested against the public 
Replay SVN repository “Revision: 132” or for those with access
to the private repository, “Revision: 2106”.

A detailed guide to creating the example\_audio core is available
on [my website](https://www.mups.co.uk/guide/fpga-replay-audio-example/)

# Hardware

More informtation about the FPGA Replay Board can be found on
the official [FPGA Replay website](http://www.fpgaarcade.com/).

# License

All code is licensed under the FPGA Arcade license. For details
please refer to the header comments of the vhd files.
