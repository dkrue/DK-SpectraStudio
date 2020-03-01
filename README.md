# DK-SpectraStudio
_SpectraStudio_ is a real-time audio visualizer with music-reactive effects. This project features a magnetically mounted light bar with 32 lensed LEDs and two side-mounted speaker displays. The light bar shows a constantly morphing color display with several audio analysis algorithms, and the side LEDs show frequency amplitude.

![Spectra Studio Visualizer Bar](/images/spectra_studio_top_red.jpg)

_The Spectra Studio project magnetically attached to the top of an iMac screen._

## About
This is one of my _[Spectra](https://github.com/search?q=user%3Adkrue+spectra)_ series audio visualizer projects. This project is not currently available in my
[Etsy Store: Circuits & Sawdust](https://www.etsy.com/shop/CircuitsAndSawdust) as it's more of an experimental work-in-progress.

## Goal
My main goal with this project was to take my existing Spectra audio analysis projects and create more of a "show" effect with 32 LEDs plus 8 LEDs mounted to each of my left & right studio monitors. This gives you an immersive effect when watching concert / festival footage similar to TVs with ambient lighting. I found 10° plastic lenses on eBay to fit over each LED, which gives the lights a tight focused effect just like actual stage lighting.

![Spectra Studio Visualizer Bar](/images/spectra_studio_top_blue.jpg)

This project also uses my latest voltage divider circuit that reads audio input voltage in the simplest way possible. You can find more about the circuit in [my blog post: Analyzing line audio input with Arduino.](https://dkrue.github.io/arduino/2020/02/09/analyzing_line_audio_input_with_arduino.html)

![Spectra Studio Circuit](/images/spectra_studio_circuit.jpg)

## How it works
This is an [Adafruit Pro Trinket](https://www.adafruit.com/product/2000) (5V) based project. The Pro Trinket is great because it's cheap but still offers an analog reference pin to tie line-level audio into. It's 16MHz and fast enough to analyze 16 bands of FFT audio, but it has no serial to USB chip, so I recommend breadboarding this project out on an Arduino Uno. You can use the serial output for debugging, and then transfer everything to the Pro Trinket. Or you could use something like the [Adafruit Metro Mini](https://www.adafruit.com/product/2590) for a similar cost.

The original inspiration for this is the [Adafruit Piccolo](https://learn.adafruit.com/piccolo/overview) project. Rather than rely on a variable-level microphone input for the source audio, I addded a 3.5mm audio input jack with voltage divider circuit to analyze line-level audio. A second 3.5mm output jack is tied to the input jack to pass through audio to other modules.  I found this [spectrum analyzer Instructables](https://www.instructables.com/id/Arduino-Spectrum-Analyzer-on-a-10x10-RGB-LED-Matri/) to be helpful for information on how to wire in audio jacks.

## Ingredients
This project uses generic Chinese parts available on eBay, aside from the Adafruit Trinket microcontroller.

- About a meter of RGB 5050 LEDs (using 32 LEDs) mounted on an angled wooden strip with magnetic strip backing
- [10° plastic lenses for SMD 5050 LEDs](https://www.ebay.com/itm/LENS-For-5050-SMD-LEDS-WS2812-WS2811-SK6812-10-30-60-120-140-Degree-Angle/292164704213?ssPageName=STRK%3AMEBIDX%3AIT&var=591105126018&_trksid=p2057872.m2749.l2649)
- [Adafruit Pro Trinket](https://www.adafruit.com/product/2000)
- Two 3.5mm headphone jacks and resistors as described in [my blog post: Analyzing line audio input with Arduino](https://dkrue.github.io/arduino/2020/02/09/analyzing_line_audio_input_with_arduino.html)

![Spectra Studio Visualizer Dark](/images/spectra_studio_all.jpg)
