//inspired by Deadlock's video on n-body simulations https://www.youtube.com/watch?v=nZHjD3cI-EU
//built in the unity engine
//C# code created by me.

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

using Unity.Mathematics;
using Unity.Burst;
using Unity.Jobs;
using Unity.Collections;

public class Sim2D : MonoBehaviour
{
    NativeArray<Body> nativeBodies;
    NativeList<Node> quadTreeNodes;
    NativeList<int> quadTreeParents;
    NativeArray<float2> accelerations;

    public Body[] bodies;
    [Header("Initialization Settings")]
    public uint numBodies = 1000;
    public float spacing = 1f;
    public float initialRadius = 0.3f;
    public float initialMass = 1f;
    public float initialSpeed = 5f;
    [Header("Bounds Settings")]
    public float2 bounds;
    public float boundsDamping = 0.85f;
    [Header("Simulation Settings")]
    public bool bodyToBodyCollisions = true;
    public bool showGizmos = false;
    public float gravitationalConstant = 1f;
    public float collisionDamping = 0.8f;
    [Header("Performance")]
    public float minDist = 0.01f;
    public float accuracyTheta = 1.0f;
    public float maxVelocity = 25f;
    [Range(1, 10)] public int subSteps = 4;
    [Range(1, 10)] public int dtReduction = 2;
    public QuadTree quadTree;

    void Awake()
    {
        nativeBodies = new NativeArray<Body>((int)numBodies, Allocator.Persistent);
        bodies = InitBodies(numBodies, spacing, initialRadius, initialMass, initialSpeed);
        HandleCollisionsAndForcesBruteForce(); //prehandle any overlapping bodies
        nativeBodies.CopyFrom(bodies);

        quadTreeNodes = new NativeList<Node>(4096, Allocator.Persistent);
        quadTreeParents = new NativeList<int>(1024, Allocator.Persistent);
        accelerations = new NativeArray<float2>((int)numBodies, Allocator.Persistent);
    }
    void OnDestroy()
    {
        //if(nativeBodies.IsCreated) 
        nativeBodies.Dispose();
        //if(quadTreeNodes.IsCreated) 
        quadTreeNodes.Dispose();
        //if(quadTreeParents.IsCreated) 
        quadTreeParents.Dispose();
        //if(accelerations.IsCreated) 
        accelerations.Dispose();
    }

    public struct Quad
    {
        public float2 center;
        public float size;
        public uint FindQuadrant(float2 p) //assigns a number between [0, 3] based on what quadtrant of this quad p is in
        {
            //***check node struct for specific quadrant numbers***
            int x = p.x > center.x ? 1 : 0;
            int y = p.y < center.y ? 1 : 0;
            return (uint)(x + y * 2);
        }
        public bool CircleIntersectQuad(float2 circleCenter, float radius) //checks if the input circle intersects with this quad
        {
            float halfSize = size * 0.5f;
            float closestX = math.clamp(circleCenter.x, center.x - halfSize, center.x + halfSize);
            float closestY = math.clamp(circleCenter.y, center.y - halfSize, center.y + halfSize);

            float distanceX = circleCenter.x - closestX;
            float distanceY = circleCenter.y - closestY;

            float sqrDistance = distanceX * distanceX + distanceY * distanceY;
            return sqrDistance < (radius * radius);
        }
        public Quad(float2 center, float size)
        {
            this.center = center;
            this.size = size;
        }
    }

    public struct QuadTree
    {
        public List<Node> nodes;
        public List<int> parents;
        public void InitTree() 
        {
            if(nodes == null) nodes = new List<Node>(4096);
            else nodes.Clear();
            if(parents == null) parents = new List<int>(1024);
            else parents.Clear();
        }
        public void Add(Node node) {nodes.Add(node);}
    }

    public struct Node
    {
        /*child quadrants:
        1 0
        3 2
        */
        //public uint[] children;
        public int firstChild; //-1 if no children
        public Quad quad;
        public int bodyIndex; //-1 if no body
        public float2 centerOfMass;
        public float mass;

        public void CreateChildren(QuadTree quadTree, out QuadTree newQuadTree)
        {
            firstChild = quadTree.nodes.Count;
            //children = new uint[4];
            float childHalfSize = quad.size/2;

            float offsetDistance = quad.size/4;
            for (int i = 0; i < 4; i++)
            {
                //assigns child to a quad number
                float signX = i % 2 == 0 ? -1 : 1;
                float signY = i < 2 ? 1 : -1;

                //offset the child a quarter of the size
                float2 offset = new float2(offsetDistance * signX, offsetDistance * signY);

                //create new child
                Quad newQuad = new Quad(quad.center + offset, childHalfSize);
                Node newNode = new Node{quad = newQuad, firstChild = -1, bodyIndex = -1};
                quadTree.Add(newNode);
            }

            newQuadTree = quadTree;
        }
        public void SetQuad(Quad val) {quad = val;}
        public bool IsBranch() => firstChild >= 0;
    }

    //initializes bodies into a square
    Body[] InitBodies(uint numBodies, float spacing, float bodyRadius, float bodyMass, float initialSpeed)
    {
        Body[] bodies = new Body[numBodies];

        int bodiesPerRow = (int)Mathf.Sqrt(numBodies);
        int bodiesPerColumn = (int)(numBodies-1)/bodiesPerRow+1;
        float mySpacing = bodyRadius*2f + spacing;

        for(int i = 0; i < numBodies; i++)
        {
            float x = (i % bodiesPerRow - bodiesPerRow / 2f + 0.5f) * mySpacing;
            float y = (i / bodiesPerRow - bodiesPerColumn / 2f + 0.5f) * mySpacing;
            float2 pos = new float2(x, y);

            //add an orbital velocity relative to (0, 0) to each body
            float2 orbitalVel = pos / math.length(pos);
            orbitalVel = new float2(-orbitalVel.y, orbitalVel.x);
            float2 initVel = orbitalVel * initialSpeed;

            Body b = new Body(bodyMass, bodyRadius, pos, initVel);
            bodies[i] = b;
        }
        return bodies;
    }

    void Startxx()
    {
        bodies = InitBodies(numBodies, spacing, initialRadius, initialMass, initialSpeed);
        quadTree.InitTree();

        for(int prePass = 0; prePass < 10; prePass++)
        {
            ConstructQuadTree(bodies, ref quadTree);
            HandleCollisions(quadTree, ref bodies);
        }
    }




// ------------- construct quadtree functions (both normal and ijob) ------------------------
    void ConstructQuadTree(Body[] bodies, ref QuadTree quadTree) //construct quadtree as a normal function
    {
        //initialize the top node and the tree
        //quadTree = new QuadTree();
        quadTree.InitTree();
        Node topNode = new Node();
        Quad topQuad = new Quad(float2.zero, Mathf.Max(bounds.x, bounds.y));
        topNode.SetQuad(topQuad);
        topNode.bodyIndex = -1;
        quadTree.Add(topNode);

        topNode.CreateChildren(quadTree, out quadTree);
        quadTree.nodes[0] = topNode;
        
        //loops over all the bodies
        int bodyIndex = 0;
        foreach(Body b in bodies)
        {
            Node myNode = topNode;
            int nodeIndex = 0;
            //loops until b finds a leaf node
            while (myNode.IsBranch())
            {
                uint childNum = myNode.quad.FindQuadrant(b.position);
                nodeIndex = myNode.firstChild + (int)childNum;
                myNode = quadTree.nodes[nodeIndex];
            }

            //checks if this leaf node has a body
            if(myNode.bodyIndex > -1)
            {
                Body otherB = bodies[myNode.bodyIndex];
                int otherBodyIndex = myNode.bodyIndex;
                uint myBQuad, otherBQuad;
                int myIndex, otherIndex;

                myNode.bodyIndex = -1;
                myNode.mass += b.mass;
                quadTree.nodes[nodeIndex] = myNode;

                uint debugCount = 0;
                int currentParentIndex = nodeIndex;
                Node currentParentNode = myNode;
                //loop until the two bodies are in different quads
                while (true)
                {
                    //divides the quadtree further
                    currentParentNode.CreateChildren(quadTree, out quadTree);
                    quadTree.nodes[currentParentIndex] = currentParentNode;
                    quadTree.parents.Add(currentParentIndex);

                    //finds and checks the quadrants of the two bodies after division
                    myBQuad = currentParentNode.quad.FindQuadrant(b.position);
                    otherBQuad = currentParentNode.quad.FindQuadrant(otherB.position);

                    myIndex = currentParentNode.firstChild + (int)myBQuad;
                    otherIndex = currentParentNode.firstChild + (int)otherBQuad;
                    //if the two bodies are in different quadrants, break this loop
                    if(myBQuad != otherBQuad) 
                    {
                        break;
                    }

                    //if not, set the current node
                    currentParentIndex = myIndex;
                    currentParentNode = quadTree.nodes[currentParentIndex];

                    //stops this loop if it exceeds 100 counts
                    debugCount ++;
                    if(debugCount > 100)
                    {
                        Debug.LogError("A tree construction attempted 100 divisions! Were 2 bodies too close?");
                        break;
                    }
                }

                //update the nodes within the quadtree struct
                Node finalMyNode = quadTree.nodes[myIndex];
                finalMyNode.bodyIndex = bodyIndex;
                finalMyNode.centerOfMass = b.position;
                finalMyNode.mass = b.mass;

                Node finalOtherNode = quadTree.nodes[otherIndex];
                finalOtherNode.bodyIndex = otherBodyIndex;
                finalOtherNode.centerOfMass = otherB.position;
                finalOtherNode.mass = otherB.mass;

                quadTree.nodes[myIndex] = finalMyNode;
                quadTree.nodes[otherIndex] = finalOtherNode;
            }
            else
            {
                //update the node within the quadtree struct
                myNode.bodyIndex = bodyIndex;
                myNode.centerOfMass = b.position;
                myNode.mass = b.mass;
                quadTree.nodes[nodeIndex] = myNode;
            }

            bodyIndex ++;
        }
    }

    [BurstCompile]
    public struct ConstructQuadTreeJob : IJob //construct quadtree as an ijob
    {
        [ReadOnly] public NativeArray<Body> bodies;
        public NativeList<Node> nodes;
        public NativeList<int> parents;
        public Quad topQuad;

        public void Execute()
        {
            nodes.Clear();
            parents.Clear();

            Node topNode = new Node() {quad = topQuad, firstChild = -1, bodyIndex = -1};
            nodes.Add(topNode);
            CreateChildren(0);

            for(int bodyIndex = 0; bodyIndex < bodies.Length; bodyIndex++)
            {
                Body b = bodies[bodyIndex];
                int nodeIndex = 0;

                //loops until b finds a leaf node
                while (nodes[nodeIndex].IsBranch())
                {
                    Node n = nodes[nodeIndex];
                    uint childNum = n.quad.FindQuadrant(b.position);
                    nodeIndex = n.firstChild + (int)childNum; 
                }

                //checks if this leaf node has a body
                Node leaf = nodes[nodeIndex];
                if(leaf.bodyIndex > -1)
                {
                    Body otherB = bodies[leaf.bodyIndex];
                    int otherBodyIndex = leaf.bodyIndex;

                    uint myBQuad, otherBQuad;
                    int myIndex, otherIndex;
                    int myFinalIndex = -1, otherFinalIndex = -1;

                    leaf.bodyIndex = -1;
                    leaf.mass += b.mass;
                    nodes[nodeIndex] = leaf;

                    int safety = 0; //counts quadtree branching
                    
                    int currentParentIndex = nodeIndex;
                    Node currentParentNode = leaf;
                    //loop until the two bodies are in different quads
                    while (true)
                    {
                        //divides the quadtree further
                        CreateChildren(currentParentIndex);
                        currentParentNode = nodes[currentParentIndex];
                        parents.Add(currentParentIndex);

                        //finds and checks the quadrants of the two bodies after division
                        myBQuad = currentParentNode.quad.FindQuadrant(b.position);
                        otherBQuad = currentParentNode.quad.FindQuadrant(otherB.position);

                        myIndex = currentParentNode.firstChild + (int)myBQuad;
                        otherIndex = currentParentNode.firstChild + (int)otherBQuad;

                        //if the two bodies are in different quadrants, break this loop
                        if(myBQuad != otherBQuad) 
                        {
                            myFinalIndex = myIndex;
                            otherFinalIndex = otherIndex;
                            break;
                        }

                        //if not, set the current node
                        currentParentIndex = myIndex;
                        currentParentNode = nodes[currentParentIndex];

                        //stops this loop if it exceeds 100 counts
                        safety ++;
                        if(safety > 100)
                        {
                            Debug.LogError("A tree construction attempted 100 divisions! Were 2 bodies too close?");
                            myFinalIndex = myIndex;
                            otherFinalIndex = otherIndex;
                            break;
                        }
                    }

                    //update the nodes within the quadtree struct
                    Node finalLeaf = nodes[myFinalIndex];
                    finalLeaf.bodyIndex = bodyIndex;
                    finalLeaf.centerOfMass = b.position;
                    finalLeaf.mass = b.mass;
                    nodes[myFinalIndex] = finalLeaf;

                    Node finalOtherNode = nodes[otherFinalIndex];
                    finalOtherNode.bodyIndex = otherBodyIndex;
                    finalOtherNode.centerOfMass = otherB.position;
                    finalOtherNode.mass = otherB.mass;
                    nodes[otherFinalIndex] = finalOtherNode;
                }
                else
                {
                    //update the node within the quadtree struct
                    leaf.bodyIndex = bodyIndex;
                    leaf.centerOfMass = b.position;
                    leaf.mass = b.mass;
                    nodes[nodeIndex] = leaf;
                }
            }
        }

        void CreateChildren(int parentIndex)
        {
            Node parent = nodes[parentIndex];
            int firstChild = nodes.Length;
            parent.firstChild = firstChild;
            nodes[parentIndex] = parent;

            float childHalfSize = parent.quad.size/2f;
            float offsetDistance = parent.quad.size/4f;

            for (int i = 0; i < 4; i++)
            {
                //assigns child to a quad number
                float signX = i % 2 == 0 ? -1 : 1;
                float signY = i < 2 ? 1 : -1;

                //offset the child a quarter of the size
                float2 offset = new float2(offsetDistance * signX, offsetDistance * signY);

                //create new child
                Quad newQuad = new Quad(parent.quad.center + offset, childHalfSize);
                Node newNode = new Node{quad = newQuad, firstChild = -1, bodyIndex = -1};
                nodes.Add(newNode);
            }
        }
    }












    //calculates the center of mass of every node after construction
    QuadTree CalculateCentersOfMass(QuadTree quadTree) //normal CoM calculations function
    {
        //loops through the tree in reverse
        //excludes all leaf nodes
        for(int i = quadTree.parents.Count - 1; i >= 0; i--)
        {
            int parentIndex = quadTree.parents[i];
            Node node = quadTree.nodes[parentIndex];

            //calculate CoM based on this node's children
            float2 totalCoM = float2.zero;
            float totalMass = 0;
            for(int j = 0; j < 4; j++)
            {
                Node child = quadTree.nodes[node.firstChild + j];

                if(child.mass <= 0) continue;

                totalCoM += child.centerOfMass * child.mass;
                totalMass += child.mass;
            }
            float2 centerOfMass = totalCoM / totalMass;
            node.mass = totalMass;
            node.centerOfMass = centerOfMass;

            quadTree.nodes[parentIndex] = node;
        }
        return quadTree;
    }

    [BurstCompile]
    public struct CalculateCentersOfMassJob : IJob //calculate CoM job function
    {
        public NativeList<Node> nodes;
        [ReadOnly] public NativeList<int> parents;

        public void Execute()
        {
            //loops through the tree in reverse
            //excludes all leaf nodes
            for(int i = parents.Length - 1; i >= 0; i--)
            {
                int parentIndex = parents[i];
                Node node = nodes[parentIndex];

                //calculate CoM based on this node's children
                float2 totalCoM = float2.zero;
                float totalMass = 0;
                for(int j = 0; j < 4; j++)
                {
                    Node child = nodes[node.firstChild + j];

                    if(child.mass <= 0) continue;

                    totalCoM += child.centerOfMass * child.mass;
                    totalMass += child.mass;
                }
                float2 centerOfMass = totalCoM / totalMass;
                node.mass = totalMass;
                node.centerOfMass = totalMass > 0 ? centerOfMass : node.centerOfMass;

                nodes[parentIndex] = node;
            }
        }
    }











    List<int> stack = new List<int>(64); //using stack in acc calc and collision calc
    float2 CalculateAcceleration(int myBodyIndex, float theta, float epsilon, QuadTree quadTree)
    {
        Body b = bodies[myBodyIndex];
        float2 bodyPos = b.position;
        float2 acc = float2.zero;

        float sqrTheta = theta * theta;
        float sqrEpsilon = epsilon * epsilon;

        stack.Clear();
        stack.Add(0);

        //traverse the tree to make distance comparisons to each node
        while(stack.Count > 0)
        {
            int nodeIndex = stack[^1];
            stack.RemoveAt(stack.Count - 1);

            Node n = quadTree.nodes[nodeIndex];

            float2 normal = n.centerOfMass - bodyPos;
            float sqrDistance = math.lengthsq(normal);

            //apply force to this body if leaf node or distance to node is sufficient
            if(!n.IsBranch() || n.quad.size * n.quad.size < sqrDistance * sqrTheta)
            {
                //calc acc influence
                float dist = Mathf.Sqrt(sqrDistance);
                float denom = sqrDistance + sqrEpsilon * dist;

                //prevent division by zero
                if(denom == 0.0f)
                {
                    //Debug.LogWarning("Division by zero attempted!");
                    denom = float.MaxValue;
                }
                denom = Mathf.Min(denom, float.MaxValue);

                acc += normal * (n.mass / denom) * gravitationalConstant;
                continue;
            }

            for(int i = 3; i >= 0; i--) //add children to the stack
            {
                stack.Add(n.firstChild + i);
            }
        }

        return acc;
    }

    [BurstCompile]
    public struct CalculateAccelerationJob : IJobParallelFor //calc accelerations in parallel
    {
        [ReadOnly] public NativeArray<Body> bodies;
        [ReadOnly] public NativeArray<Node> nodes;
        public NativeArray<float2> accelerations;
        public float theta;
        public float epsilon;
        public float gravitationalConstant;

        const int stackCapacity = 128;

        public void Execute(int index)
        {
            Body b = bodies[index];
            float2 bodyPos = b.position;
            float2 acc = float2.zero;

            float sqrTheta = theta * theta;
            float sqrEpsilon = epsilon * epsilon;

            var stack = new NativeList<int>(stackCapacity, Allocator.Temp);
            stack.Add(0);

            while(stack.Length > 0)
            {
                int nodeIndex = stack[^1];
                stack.RemoveAt(stack.Length - 1);

                //stack.RemoveAt(stack.Count - 1);

                Node n = nodes[nodeIndex];

                float2 normal = n.centerOfMass - bodyPos;
                float sqrDistance = math.lengthsq(normal);

                //apply force to this body if leaf node or distance to node is sufficient
                if(!n.IsBranch() || n.quad.size * n.quad.size < sqrDistance * sqrTheta)
                {
                    //calc acc influence
                    float dist = math.sqrt(sqrDistance);
                    float denom = sqrDistance + sqrEpsilon * dist;

                    //prevent division by zero
                    if(denom == 0.0f) denom = float.MaxValue;
                    denom = Mathf.Min(denom, float.MaxValue);

                    acc += normal * (n.mass / denom) * gravitationalConstant;
                    continue;
                }

                for(int i = 3; i >= 0; i--) //add children to the stack
                {
                    stack.Add(n.firstChild + i);
                }
            }

            accelerations[index] = acc;
            stack.Dispose();
        }
    }









    //----------- \ update positions / --------------
    void UpdateAllBodiesPos(float dt)
    {
        if(bounds.x <= initialRadius*2 || bounds.y <= initialRadius * 2)
        {
            Debug.Log("Bounds is too small!");
            return;
        }

        for(int i = 0; i < bodies.Length; i++)
        {
            Body b = bodies[i];

            //update position and velocity
            b.velocity += b.acceleration * dt;

            float2 ClampMagnitude(float2 vec, float clampVal) //clamps a vector to a length
            {
                float2 normal = vec / math.length(vec);
                return normal * math.clamp(math.length(vec), 0, clampVal);
            }

            b.velocity = ClampMagnitude(b.velocity, maxVelocity == 0 ? 1 : maxVelocity); //safety net for eratic velocity
            b.position += b.velocity * dt;
            b.acceleration = float2.zero;

            //resolve border collision
            if(Mathf.Abs(b.position.x) > bounds.x / 2f - b.radius)
            {
                b.position.x = (bounds.x / 2f - b.radius) * Mathf.Sign(b.position.x);
                b.velocity.x *= -boundsDamping;
            }
            if(Mathf.Abs(b.position.y) > bounds.y / 2f - b.radius)
            {
                b.position.y = (bounds.y / 2f - b.radius) * Mathf.Sign(b.position.y);
                b.velocity.y *= -boundsDamping;
            }

            //update the main array
            bodies[i] = b;
        }
    }

    [BurstCompile]
    public struct UpdatePositionsJob : IJobParallelFor
    {
        public NativeArray<Body> bodies;
        [ReadOnly] public NativeArray<float2> accelerations;
        public float dt;
        public float2 bounds;
        public float boundsDamping;
        public float maxVelocity;

        public void Execute(int index)
        {
            Body b = bodies[index];
            float2 acc = accelerations[index];

            //update position and velocity
            b.velocity += acc * dt;

            float2 ClampMagnitude(float2 vec, float max) //clamps a vector to a length
            {
                float2 normal = vec / math.length(vec);
                return normal * math.clamp(math.length(vec), 0, max);
            }

            b.velocity = ClampMagnitude(b.velocity, maxVelocity == 0 ? 1 : maxVelocity); //safety net for eratic velocity
            b.position += b.velocity * dt;
            b.acceleration = float2.zero;

            //resolve border collision
            if(Mathf.Abs(b.position.x) > bounds.x / 2f - b.radius)
            {
                b.position.x = (bounds.x / 2f - b.radius) * Mathf.Sign(b.position.x);
                b.velocity.x *= -boundsDamping;
            }
            if(Mathf.Abs(b.position.y) > bounds.y / 2f - b.radius)
            {
                b.position.y = (bounds.y / 2f - b.radius) * Mathf.Sign(b.position.y);
                b.velocity.y *= -boundsDamping;
            }

            //update the main array
            bodies[index] = b;
        }
    }









    //----------------\ collision handlers /--------------------

    void HandleCollisionsAndForcesBruteForce() //brute force collisions and gravity checks
    {
        for(int i = 0; i < bodies.Length; i++)
        {
            Body bodyA = bodies[i];
            for(int j = i + 1; j < bodies.Length; j++)
            {
                Body bodyB = bodies[j];

                //find the normal of the two bodies
                float2 normal = bodyA.position - bodyB.position;
                float sqrDist = math.lengthsq(normal);
                float combinedRadius = bodyA.radius + bodyB.radius;



                //handles calculating gravitational forces
                float2 uNormal = normal / math.length(normal);

                float2 forceA = gravitationalConstant * bodyB.mass / Mathf.Max(sqrDist, minDist) * -uNormal;
                bodyA.acceleration += forceA;
                float2 forceB = gravitationalConstant * bodyA.mass / Mathf.Max(sqrDist, minDist) * uNormal;
                bodyB.acceleration += forceB;




                if(!bodyToBodyCollisions) continue;
                //checks if the two bodies are colliding
                if(sqrDist <= combinedRadius * combinedRadius)
                {
                    float dist = Mathf.Sqrt(sqrDist);
                    
                    //correct velocities
                    float2 unitNormal = normal / dist;

                    float scalarNormalA = unitNormal.x * bodyA.velocity.x + unitNormal.y * bodyA.velocity.y;
                    float scalarNormalB = unitNormal.x * bodyB.velocity.x + unitNormal.y * bodyB.velocity.y;

                    float scalarNormalAPrime = (scalarNormalA * (bodyA.mass - bodyB.mass) + 2 * bodyB.mass * scalarNormalB) / (bodyA.mass + bodyB.mass);
                    float scalarNormalBPrime = (scalarNormalB * (bodyB.mass - bodyA.mass) + 2 * bodyA.mass * scalarNormalA) / (bodyA.mass + bodyB.mass);

                    bodyA.velocity += (scalarNormalAPrime - scalarNormalA) * unitNormal;
                    bodyB.velocity += (scalarNormalBPrime - scalarNormalB) * unitNormal;

                    //correct positions
                    float depth = combinedRadius - dist;

                    float totalMass = bodyA.mass + bodyB.mass;
                    bodyA.position += unitNormal * depth * bodyB.mass / totalMass;
                    bodyB.position -= unitNormal * depth * bodyA.mass / totalMass;
                    bodies[j] = bodyB;
                }
            }
            bodies[i] = bodyA;
        }
    }

    List<int> potentialCollisionIndices = new List<int>(128);
    void HandleCollisions(QuadTree quadTree, ref Body[] bodies) //normal collision handling function
    {
        if(!bodyToBodyCollisions) return;

        const float correctionPercent = 0.5f;

        for(int i = 0; i < bodies.Length; i++)
        {
            Body b = bodies[i];
            potentialCollisionIndices.Clear();
            stack.Clear();
            stack.Add(0);

            //traverse the tree and see which nodes the body intersects with
            while(stack.Count > 0)
            {
                //find the node we want to check
                int nodeIndex = stack[^1];
                stack.RemoveAt(stack.Count - 1);
                Node parent = quadTree.nodes[nodeIndex];

                //add to potential collisions if this node has a body and is a leaf
                if(!parent.IsBranch() && parent.bodyIndex > -1 && parent.bodyIndex != i) 
                {
                    potentialCollisionIndices.Add(parent.bodyIndex);
                    continue;
                }
                if(!parent.IsBranch()) continue;

                //check each child
                //Debug.Log(parent.children[0]);
                for(int j = 0; j < 4; j++)
                {
                    int childIndex = parent.firstChild + j;
                    Node child = quadTree.nodes[childIndex];

                    //if the body isn't intersecting this child quad, ignore the child
                    if(!child.quad.CircleIntersectQuad(b.position, b.radius))
                    {
                        continue;
                    }

                    //add child to stack
                    stack.Add(childIndex);
                }
            }

            //loops through potential collisions to check/solve for them
            foreach(int index in potentialCollisionIndices)
            {
                int otherBodyIndex = index;
                Body otherBody = bodies[otherBodyIndex];

                //find the normal of the two bodies
                float2 normal = b.position - otherBody.position;
                float sqrDistance = math.lengthsq(normal);
                float combinedRadius = b.radius + otherBody.radius;

                if(sqrDistance <= combinedRadius * combinedRadius)
                {
                    //solve the collision
                    float dist = Mathf.Sqrt(sqrDistance);
                    if(dist == 0) continue; //safety for division by zero
                    
                    //correct velocities
                    float2 unitNormal = normal / dist;

                    float scalarNormalA = unitNormal.x * b.velocity.x + unitNormal.y * b.velocity.y;
                    float scalarNormalB = unitNormal.x * otherBody.velocity.x + unitNormal.y * otherBody.velocity.y;

                    float scalarNormalAPrime = (scalarNormalA * (b.mass - otherBody.mass) + 2 * otherBody.mass * scalarNormalB) / (b.mass + otherBody.mass);
                    float scalarNormalBPrime = (scalarNormalB * (otherBody.mass - b.mass) + 2 * b.mass * scalarNormalA) / (b.mass + otherBody.mass);

                    b.velocity += (scalarNormalAPrime - scalarNormalA) * unitNormal * collisionDamping;
                    otherBody.velocity += (scalarNormalBPrime - scalarNormalB) * unitNormal * collisionDamping;

                    //correct positions
                    float depth = combinedRadius - dist;

                    float totalMass = b.mass + otherBody.mass;
                    b.position += unitNormal * depth * otherBody.mass / totalMass * correctionPercent;
                    otherBody.position -= unitNormal * depth * b.mass / totalMass * correctionPercent;

                    //update other body immediately
                    bodies[otherBodyIndex] = otherBody;
                }
            }

            bodies[i] = b;
        }
    }

    [BurstCompile]
    public struct HandleCollisionsJob : IJob
    {
        public NativeArray<Body> bodies;
        [ReadOnly] public NativeArray<Node> nodes;
        public float collisionDamping;
        const float correctionPercent = 0.5f;
        const int stackCapacity = 128;
        const int potentialCapacity = 256;

        public void Execute()
        {
            var stack = new NativeList<int>(stackCapacity, Allocator.Temp);
            var potential = new NativeList<int>(potentialCapacity, Allocator.Temp);

            for(int i = 0; i < bodies.Length; i++)
            {
                Body b = bodies[i];
                stack.Clear();
                potential.Clear();
                stack.Add(0);

                //traverse the tree and see which nodes the body intersects with
                while(stack.Length > 0)
                {
                    //find the node we want to check
                    int nodeIndex = stack[^1];
                    stack.RemoveAt(stack.Length - 1);
                    Node parent = nodes[nodeIndex];

                    //add to potential collisions if this node has a body and is a leaf
                    if(!parent.IsBranch()) 
                    {
                        if(parent.bodyIndex > -1 && parent.bodyIndex != i) 
                            potential.Add(parent.bodyIndex);
                        continue;
                    }

                    //check each child
                    for(int j = 0; j < 4; j++)
                    {
                        int childIndex = parent.firstChild + j;
                        Node child = nodes[childIndex];

                        //if the body isn't intersecting this child quad, ignore the child
                        if(!child.quad.CircleIntersectQuad(b.position, b.radius)) continue;

                        //add child to stack
                        stack.Add(childIndex);
                    }
                }

                //loops through potential collisions to check/solve for them
                for(int k = 0; k < potential.Length; k++)
                {
                    int otherBodyIndex = potential[k];
                    Body otherBody = bodies[otherBodyIndex];

                    //find the normal of the two bodies
                    float2 normal = b.position - otherBody.position;
                    float sqrDistance = math.lengthsq(normal);
                    float combinedRadius = b.radius + otherBody.radius;

                    if(sqrDistance <= combinedRadius * combinedRadius)
                    {
                        //solve the collision
                        float dist = Mathf.Sqrt(sqrDistance);
                        if(dist == 0) continue; //safety for division by zero
                        
                        //correct velocities
                        float2 unitNormal = normal / dist;

                        float scalarNormalA = unitNormal.x * b.velocity.x + unitNormal.y * b.velocity.y;
                        float scalarNormalB = unitNormal.x * otherBody.velocity.x + unitNormal.y * otherBody.velocity.y;

                        float scalarNormalAPrime = (scalarNormalA * (b.mass - otherBody.mass) + 2 * otherBody.mass * scalarNormalB) / (b.mass + otherBody.mass);
                        float scalarNormalBPrime = (scalarNormalB * (otherBody.mass - b.mass) + 2 * b.mass * scalarNormalA) / (b.mass + otherBody.mass);

                        b.velocity += (scalarNormalAPrime - scalarNormalA) * unitNormal * collisionDamping;
                        otherBody.velocity += (scalarNormalBPrime - scalarNormalB) * unitNormal * collisionDamping;

                        //correct positions
                        float depth = combinedRadius - dist;

                        float totalMass = b.mass + otherBody.mass;
                        b.position += unitNormal * depth * otherBody.mass / totalMass * correctionPercent;
                        otherBody.position -= unitNormal * depth * b.mass / totalMass * correctionPercent;

                        //update other body immediately
                        bodies[otherBodyIndex] = otherBody;
                    }
                }

                bodies[i] = b;
            }

            stack.Dispose();
            potential.Dispose();
        }
    }












    bool paused = false;

    //---------------- \ OLD UPDATE FUNCTION / -----------------------
    void Updatexx()
    {
        //pause the main loop if p is pressed
        if(Input.GetKeyDown(KeyCode.P)) paused = paused ? false : true;
        if(paused) return;

        //HandleCollisionsAndForcesBruteForce(); //OLD CODE
        //build a new quadtree
        ConstructQuadTree(bodies, ref quadTree);

        //calc the center of masses for each node
        quadTree = CalculateCentersOfMass(quadTree);

        //applies acceleration onto each body
        for(int i = 0; i < bodies.Length; i++)
        {
            float epsilon = 1;
            float2 acc = CalculateAcceleration(i, accuracyTheta, epsilon, quadTree);
            bodies[i].acceleration = acc;
        }

        //divides deltaTime into steps
        int steps = Mathf.Max(1, subSteps);
        float subDt = Time.deltaTime / (float)steps;
        subDt /= dtReduction;

        for(int step = 0; step < steps; step++) //check collisions in set steps
        {
            //handle collisions
            const int collisionIterations = 2;
            for(int i = 0; i < collisionIterations; i++)
            {
                HandleCollisions(quadTree, ref bodies);
            }

            //updates the bodies' positions
            UpdateAllBodiesPos(subDt);
        }
    }


    //------------------- \ NEW UPDATE FUNCTION / ----------------------------
    void Update()
    {
        if(Input.GetKeyDown(KeyCode.P)) paused = !paused;
        if(paused) return;

        Quad topQuad = new Quad(float2.zero, Mathf.Max(bounds.x, bounds.y));

        var treeJob = new ConstructQuadTreeJob
        {
            bodies = nativeBodies,
            nodes = quadTreeNodes,
            parents = quadTreeParents,
            topQuad = topQuad
        };
        JobHandle treeHandle = treeJob.Schedule();

        var comCalcJob = new CalculateCentersOfMassJob
        {
            nodes = quadTreeNodes,
            parents = quadTreeParents
        };
        JobHandle comHandle = comCalcJob.Schedule(treeHandle);

        var gravityJob = new CalculateAccelerationJob
        {
            bodies = nativeBodies,
            nodes = quadTreeNodes.AsDeferredJobArray(),
            accelerations = accelerations,
            theta = accuracyTheta,
            epsilon = 1f,
            gravitationalConstant = gravitationalConstant
        };
        JobHandle gravityHandle = gravityJob.Schedule(nativeBodies.Length, 64, comHandle);

        JobHandle lastHandle = gravityHandle;
        if (bodyToBodyCollisions)
        {
            var collisionJob = new HandleCollisionsJob
            {
                bodies = nativeBodies,
                nodes = quadTreeNodes.AsDeferredJobArray(),
                collisionDamping = collisionDamping
            };

            int steps = Mathf.Max(1, subSteps);
            for(int i = 0; i < steps; i++)
            {
                lastHandle = collisionJob.Schedule(lastHandle);
            }
        }

        var integrateJob = new UpdatePositionsJob
        {
            bodies = nativeBodies,
            accelerations = accelerations,
            dt = Time.deltaTime / dtReduction,
            bounds = bounds,
            boundsDamping = boundsDamping,
            maxVelocity = maxVelocity
        };
        JobHandle integrateHandle = integrateJob.Schedule(nativeBodies.Length, 64, lastHandle);

        integrateHandle.Complete();

        nativeBodies.CopyTo(bodies);
    }












    void OnDrawGizmos() //gizmos visualization of the quadtree
    {
        if(!showGizmos) return;

        //draw the quadtree
        for(int i = 0; i < quadTreeNodes.Length; i++)
        {
            Node node = quadTreeNodes[i];

            Vector2 nodeCenter = new Vector2(node.quad.center.x, node.quad.center.y);
            Gizmos.DrawWireCube(nodeCenter, new Vector3(node.quad.size, node.quad.size, 1f));

            Vector2 nodeCoM = new Vector2(node.centerOfMass.x, node.centerOfMass.y);
            Gizmos.DrawWireSphere(nodeCoM, 0.1f);
        }
    }
}

[System.Serializable]
public struct Body //body contains a mass, a radius, a position, a velocity, and an acceleration
{
    public float mass;
    public float radius;
    public float2 position;
    public float2 velocity;
    public float2 acceleration;
    public Body(float m, float rad, float2 pos, float2 vel)
    {
        mass = m;
        radius = rad;
        position = pos;
        velocity = vel;
        acceleration = float2.zero;
    }
}

