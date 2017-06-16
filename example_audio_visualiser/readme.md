# Audio Visualiser

This example builds upon the "example_audio" core.

It demonstrates use of the FPGA's Block RAM (1024x18) to store
720 samples worth of L/R 8bit audio and display as a simple visualiser.

A new ram_d1024_w18 was derived from the existing ram_d2048_w8 to enable
16bits of data (8bit per channel).

The remainder of the new additions are withing the "Video Sync" section of
the code. With a minor change to p_audio_out to append each new sample into
a FIFO.

The FIFO_ASync_D16 allows samples obtained in the audio clock domain
to be written to memory based on the video clock.


