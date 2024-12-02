# PlatStar

A platform-based path finding library! Useful for 2D platforming games.
(3D Coming Soon)

Basically, when AI navigatiom for a platformer you need to take into account that
- The positions an agent can occupy are a collection of continuous spaces (the platforms they stand on)
- Some links between platforms might not be accessible by certain agents (for example, the agent can't jump high enough)
A basic node-is-position graph isn't sufficient for such a task.

So, this crate gives you the `PlatStar` type, on which you store your platforms and links to perform shortest path queries and the like. 
This and other abstractions provided by this crate are designed to be extendible, so you can customise them to fit your game without having to worry about implementing any graph algorithms.

Using a popular game engine?
Have a look at the official PlatStar plugin for that engine:
- Godot: (Coming Soon)
- Bevy: (Coming Soon)