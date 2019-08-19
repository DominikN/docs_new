---
id: rosbot-and-ws2812b-led-signalization
sidebar_label: 2. ROSbot with WS2812B LEDs signalization 
title: ROSbot with WS2812B LEDs signalization
---

> This tutorial requires a new, Mbed OS version of the ROSbot firmware!

## Introduction

<a href="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img1.jpg" data-fancybox="images" data-caption="ROSbot status illumination">
    <img src="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img1-small.jpg" alt="ROSbot status illumination" class="hover-shadow"/>
</a>

The ROSbot by default has three LEDs that can be used by user's applications to indicate status and battery condition. However, their location at the rear panel and the fact that they have only single color can limit possible use cases. In more demanding applications where there are many distinct robot's states we need more clear, visible and robust solution.  

<p align="center">
<img alt="WS2812B chip" src="/docs/assets/img/mbed-tutorials/ws2812b-chip.jpg" title="WS2812B chip" />
</p>

In this tutorial we will present you with elegant and ... approach to this problem. We will use a popular and cheap ws2812b RGB LED strip. Each LED pixel has IC and is capable of displaying 256 levels of brightness for each color giving overall 16 millions of colors depth.
The ws2812b IC utilizes the NZR protocol with 800Kbit/s speed which allows high refresh rates.
### Prerequisites

## Hardware installation

## WS2812B library for CORE2

## Enabling signalization support in ROSbot firmware

<a data-fancybox href="#feature-sample">
    <img src="/docs/assets/img/mbed-tutorials/mbed-tutorial2-still-frame.png" width="360px" class="hover-shadow"/>
</a>

<video width="640" height="320" controls id="feature-sample" style="display:none;">
    <source src="/docs/assets/video/mbed-tutorials/feature_sample.webm" type="video/webm">
    Your browser doesn't support HTML5 video tag.
</video>
