Juan Manzo
Phong_Frgamnet_Shader:
First i computed the ambient value by multiplying the elements of ka by the elements of the ambient light intensity.
Next is the diffuse value. I first computed the radius then computed the light intensity divided by the radius. Then i took that vector and multiplied its elemnts by the elements of the kd coefficient.Then i multiplied it by the max of 0 and the dot product of the normal and light position vectors. Lastly is the specular value. First i multiplied the ks coefficient by the intesity vector, computed the bisector of v and l to compute the h value. Then i took the dot product of h and the normal vector and raised that value by p and multiplied it by the other value computed earlier. then i added the ambient specular and diffuse to get the color.

For the texture_fragment_shader, the same process as the phong function was done but my kd coefficient this time is gotten from the getColor function.
For the rasterizer function i computed all the interpolated values using the alpha beta and gamma values from the computeBarycentric function and multiplied it by the corresponding values.