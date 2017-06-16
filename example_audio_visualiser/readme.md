# Audio Visualiser

This example builds upon the "example\_audio" core.

It demonstrates use of the FPGA's Block RAM (1024x18) to store
720 samples worth of L/R 8bit audio and display as a simple visualiser.

A new ram\_d1024\_w18 was derived from the existing ram\_d2048\_w8 to enable
16bits of data (8bit per channel).

The majority of the new additions are within the "Video Sync" section of
the code. With a minor change to p\_audio\_out to append each new sample into
a FIFO.

The FIFO\_ASync\_D16 allows samples obtained in the audio clock domain
to be written to memory based on the video clock.


