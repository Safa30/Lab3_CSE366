# PACMAN

## Overview

Pac3man is a Python 3 adaptation of the popular Berkeley Pacman assignments. This project provides an educational framework for understanding and implementing key search algorithms in artificial intelligence, such as DFS, BFS, UCS, and A*. These assignments are designed to help students learn problem-solving strategies and explore AI concepts interactively.

In addition to the Pacman assignments, the repository also includes:

<ul>
<li>Markov Babbler: A simple text generator using Markov chains.</li>
<li>Naive Bayesian Spam Classifier: A spam detection tool based on naive Bayesian classification.</li>
</ul>

## Maze Search Algorithm Performance

Below is a performance comparison of different search algorithms (BFS, DFS, UCS, A*) across three maze configurations (tinyMaze, mediumMaze, bigMaze). The table includes the hypothetical runtime and path length data for each algorithm, which should be replaced with actual values after running the commands.

### TinyMaze

| Algorithm | Time (s) | Length of Path |
|-----------|----------|----------------|
| BFS       | 0.00041     | 8              |
| DFS       | 0.00076     | 10              |
| UCS       | 0.00083     | 8              |
| A*        | 0.00061     | 8              |

![tinyMaze](img/tinyMaze.png)

### MediumMaze

| Algorithm | Time (s) | Length of Path |
|-----------|----------|----------------|
| BFS       | 0.00419     | 68             |
| DFS       | 0.00226     | 130            |
| UCS       | 0.00900     | 68             |
| A*        | 0.00702     | 68             |

![mediumMaze](img/mediumMaze.png)

### BigMaze

| Algorithm | Time (s) | Length of Path |
|-----------|----------|----------------|
| BFS       | 0.00808     | 210            |
| DFS       | 0.00519     | 210            |
| UCS       | 0.04082     | 210            |
| A*        | 0.03424     | 210            |

![bigMaze](img/bigMaze.png)

Run the following commands to populate the tables with actual results:

#### tinyMaze
<ul>
<li>python pacman.py -l tinyMaze -p SearchAgent -a fn=bfs</li>
<li>python pacman.py -l tinyMaze -p SearchAgent -a fn=dfs</li>
<li>python pacman.py -l tinyMaze -p SearchAgent -a fn=ucs</li>
<li>python pacman.py -l tinyMaze -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic</li>
</ul>

#### mediumMaze
<ul>
<li>python pacman.py -l mediumMaze -p SearchAgent -a fn=bfs</li>
<li>python pacman.py -l mediumMaze -p SearchAgent -a fn=dfs</li>
<li>python pacman.py -l mediumMaze -p SearchAgent -a fn=ucs</li>
<li>python pacman.py -l mediumMaze -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic</li>
</ul>

#### bigMaze
<ul>
<li>python pacman.py -l bigMaze -p SearchAgent -a fn=bfs</li>
<li>python pacman.py -l bigMaze -p SearchAgent -a fn=dfs</li>
<li>python pacman.py -l bigMaze -p SearchAgent -a fn=ucs</li>
<li>python pacman.py -l bigMaze -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic</li>
</ul>

