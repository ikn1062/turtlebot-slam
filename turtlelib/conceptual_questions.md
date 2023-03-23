# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- rigid2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input
- diff_drive - Handles kinematics of the differential drive robot

# Conceptual Questions
1. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
   - Which of the methods would you implement and why?
        1. Create a function that has an input of a Vector2D object, calculates the normal using the x and y components, and returns a new Vector2D with the new normalized x and y. This can be outside the struct definition of Vector2D, but will require management of inputs and outputs.
        2. You could calculate the normal of the components and divide x and y seperately whenever is needed, and create and apply a division operator for the Vector2D object. This reduces overhead for specific vectorization, and keeps a generalized divisor operation.
        3. A vectorize() call within the struct, which takes the x and y components of the vector, calculates the normal, and does not return any values. This needs to be implemented inside the struct itself, and does not require management of input and output. I choose this implementation because of the lack of management of additional output from vectorize().



2. What is the difference between a class and a struct in C++?
The differences between classes and structs are related to the object members' default visibility. The default visibility for a class is private, where other objects and functions cannot directly access members/methods of the class without explicity being declared as public. The opposite is true for a struct, where all members are public by default.


3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?
C.2: Use class if the class has an invariant; use struct if the data members can vary independently
C.8: Use class rather than struct if any member is non-public
- An invariant exists in Transform2D as the members of the class are used to calculations and need to stay constant and relative to each other given the initialized values, the members of Vector2D do not, and therefore can vary independantly. 

C.3: Represent the distinction between an interface and an implementation using a class
- The Transform2D acts as an interface to a class that can change the representation of spatial coordinates, and therefore should remain as a class. Vectors do not need to have any hidden functions, and can be left as a struct.


4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?
C.46: By default, declare single-argument constructors explicit
- The constructor only needs a single arguement and thereofore are explicity declared, avoiding unintended implicit conversions.


5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constxants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

- Inverse is a function used to calculate and return a new Transform2D object, without altering the initial Transform2D input provided. 
- The operator*=() is used to multiply the current transform object with the object to the rhs, and therefore should not be declared with the const decleration.
- The use of const is to allow for readability within the code, to note which objects are not altered in functions, and raise errors if any function tried to alter these values.

