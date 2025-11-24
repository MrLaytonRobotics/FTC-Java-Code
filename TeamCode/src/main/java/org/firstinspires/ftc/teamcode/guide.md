# FTC Team Code Guide

Welcome to our team’s coding space!  
This guide explains how we organize our robot code, where you should write your code, and how to keep everything working smoothly.

---

## 1. Where Our Code Lives

All robot code is inside:

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode
```

Project structure:

```
production/
  teleop/         → Final TeleOp code used in matches
  autonomous/     → Final Autonomous code used in matches

members/
  yourname/       → Your personal practice + experiment folder

subsystems/       → Shared robot hardware classes (drive, arm, etc.)
utils/            → Helper classes (math, timers, etc.)
```

---

## 2. Your Folder: Work Safely

Each student has their own folder inside `members/`. Example:

```
members/om/
members/ishani/
members/prathika/
members/vishwath/
members/saksham/
members/adishesh/
members/shreyank/
members/bala/
```

**All your code goes inside your own folder.**

You may create:
- OpModes  
- helper classes  
- experiments  
- prototypes  

Just keep everything in your folder.

---

## 3. What Goes in “production”

Only robot-ready code goes into:

```
production/teleop/
production/autonomous/
```

This is the code the robot actually runs.

You do **not** move code into production.  
Only the coach or lead developer moves stable code here.

---

## 4. Git Branches: Which One to Use

We use two branches:

- **main** – final, stable, robot-ready code  
- **dev** – development branch (everyone works here)

**Always use the `dev` branch.**

---

## 5. Rules for Committing Code

Before committing and pushing:

### ✔️ Rule 1 — Your code must compile  
No red errors.

### ✔️ Rule 2 — Only change files in your own folder  
Example:
```
members/yourname/MyFirstOpMode.java
```

### ❌ Do NOT edit:
- other students’ folders  
- production folders  
- subsystems/  
- utils/

### ❌ Never push broken code

---

## 6. Running Your Code on the Robot

Only **one laptop** connects to the robot (the “robot laptop”).


