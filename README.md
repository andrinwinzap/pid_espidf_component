# ESP-IDF PID Component

A simple and efficient PID controller for ESP-IDF with optional feedforward support.

## Features

- PID + feedforward (`kf`) control
- Easy integration in control loops
- Minimal and fast, ideal for real-time tasks

## Installation

```bash
cd your-esp-idf-project
git submodule add https://github.com/andrinwinzap/pid_espidf_component.git components/pid
```

## Usage

```c
#include "pid.h"

pid_controller_t pid;
pid_init(&pid, 1.0f, 0.5f, 0.1f, 0.0f);

float output = pid_update(&pid, target, measured, dt);
```

## API

```c
void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float kf);
float pid_update(pid_controller_t *pid, float target, float measured, float dt);
```
