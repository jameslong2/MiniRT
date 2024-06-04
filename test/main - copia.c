#include "mlx.h"
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#define PI 22/7
#define EPSILON 0.000001
typedef struct {
    float x, y, z;
} Vector3;

typedef struct {
    float x, y;
} Vector2;

typedef struct {
    float x, y, z, w;
} Quaternion;

typedef struct {
    Vector3 position;
    Quaternion rotation;
    float near_plane;
    float far_plane;
    float fov;
} Camera;

typedef struct {
    int *triangles;
    Vector3 *vertices;
    Vector2 *uvs;
    int vert_count;
    int tri_count;
    Vector3 color;
} Object;

typedef struct {
    Vector3 position;
    float radius;
    float intensity;
} PointLight;

typedef struct {
    void *mlx;
    void *win;
    void *img;
    int		bits_per_pixel;
	int		line_length;
	int		endian;
    char *addr;
    int width;
    int height;
    int *framebuffer;
} Renderer;
// Utility functions for vector operations
Vector3 vector_add(Vector3 a, Vector3 b) {
    return (Vector3){a.x + b.x, a.y + b.y, a.z + b.z};
}

Vector3 vector_sub(Vector3 a, Vector3 b) {
    return (Vector3){a.x - b.x, a.y - b.y, a.z - b.z};
}

Vector3 vector_scale(Vector3 v, float scale) {
    return (Vector3){v.x * scale, v.y * scale, v.z * scale};
}

float vector_dot(Vector3 a, Vector3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
Vector3 vector_mul(Vector3 a, Vector3 b) {
    return (Vector3){a.x * b.x , a.y * b.y , a.z * b.z};
}
Vector3 vector_cross(Vector3 a, Vector3 b) {
    return (Vector3){
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

Vector3 vector3_scale_add(Vector3 a, float scale, Vector3 b) {
    Vector3 scaled_a = vector_scale(a, scale);
    return vector_add(scaled_a, b);
}

Vector3 vector_normalize(Vector3 v) {
    float length = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
    return (Vector3){v.x / length, v.y / length, v.z / length};
}

typedef struct {
    Vector3 origin;
    Vector3 direction;
} Ray;

typedef struct {
    int hit;
    float distance;
    Vector3 point;
    Vector3 normal;
    Vector2 uv;
    float tnear;
    Vector2 coords;
} Intersection;
float vector_length(Vector3 v) {
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}
float vector_area(Vector3 a, Vector3 b, Vector3 c) {
    Vector3 ab = vector_sub(b, a);
    Vector3 ac = vector_sub(c, a);
    return vector_length(vector_cross(ab, ac)) * 0.5f;
}
float clamp(float value, float min, float max) {
    return fmax(min, fmin(value, max));
}

Ray create_primary_ray(const Camera *camera, int x, int y, int width, int height);
Vector2 get_texture_coordinates(const Vector3 hit_point, const Vector3 v0, const Vector3 v1, const Vector3 v2, const Vector2 uv0, const Vector2 uv1, const Vector2 uv2);
///////
Vector2 get_texture_coordinates(const Vector3 hit_point, const Vector3 v0, const Vector3 v1, const Vector3 v2, const Vector2 uv0, const Vector2 uv1, const Vector2 uv2) {
    // Compute the areas and factors
    float area_total = vector_area(v0, v1, v2);
    float area0 = vector_area(hit_point, v1, v2) / area_total;
    float area1 = vector_area(hit_point, v2, v0) / area_total;
    float area2 = vector_area(hit_point, v0, v1) / area_total;

    // Interpolate the UV coordinates using the barycentric coordinates
    Vector2 uv;
    uv.x = uv0.x * area0 + uv1.x * area1 + uv2.x * area2;
    uv.y = uv0.y * area0 + uv1.y * area1 + uv2.y * area2;
    
    return uv;
}
// calculo interseccion
Intersection rayIntersectsTriangle(Vector3 og, Vector3 dir, Vector3 vert[]){
    Intersection result;
    Vector3 vert0=vert[0];
    Vector3 vert1=vert[1];
    Vector3 vert2=vert[2];
    Vector3 edge1 = vector_sub(vert1, vert0);
    Vector3 edge2 = vector_sub(vert2, vert0);
    Vector3 h = vector_cross(dir, edge2);
    float a = vector_dot(edge1, h);
    if (a > -EPSILON && a < EPSILON){
        result.hit = 0;
        return (result);
    }
    float f = 1.0 / a;
    Vector3 s = vector_sub(og, vert0);
    float u = f * vector_dot(s, h);
    if (u < 0.0 || u > 1.0){
        result.hit = 0;
        return (result);
    }
    Vector3 q = vector_cross(s, edge1);
    float v = f * vector_dot(dir, q);
    if (v < 0.0 || u + v > 1.0) {
        result.hit = 0;
        return (result);
    }
    float t = f * vector_dot(edge2, q);
    if (t > EPSILON){
        Vector3 point = vector3_scale_add(dir, t, og);
        result.hit = 1;
        result.point = point;
        result.distance = t;
        Vector3 normal = vector_add(vector_add(vector_scale(vert0, 1 - u - v), vector_scale(vert1, u)), vector_scale(vert2, v));
        //
        //Vector3 normal = vector_cross(edge1, edge2);
        //
        result.normal = vector_normalize(normal);
        result.uv = (Vector2){u, v};
        result.tnear = t;
        return (result);
    }else{
        result.hit = 0;
        return (result);
    }
}
//
Intersection intersect_ray_object(Ray ray, Object *object) {
    // Intersection logic (e.g., Möller–Trumbore algorithm) should be implemented here
    int triangle_count = object->tri_count;
    Vector3 vertex[3];
    Intersection closest_intersection = {0};
    float closest_distance = INFINITY;
    for (int i = 0; i < triangle_count/3; i++) {
        
        int idx1 = object->triangles[i * 3];
        int idx2 = object->triangles[i * 3 + 1];
        int idx3 = object->triangles[i * 3 + 2];
        vertex[0] = object->vertices[idx1];
        vertex[1] = object->vertices[idx2];
        vertex[2] = object->vertices[idx3];
        Intersection inter = rayIntersectsTriangle(ray.origin,vector_normalize(ray.direction),vertex);
        if (inter.hit == 1 && inter.distance < closest_distance)
        {
            closest_intersection = inter;
            closest_distance = inter.distance;
        }
    }
    return closest_intersection;
}
float vector_magnitude(Vector3 v) {
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}
float vector3_distance(Vector3 a, Vector3 b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}


//
Vector3 illuminate(Ray ray, Intersection intersection, PointLight light, Object obj) {
    Vector3 hitPoint = intersection.point; 
    Vector3 hitNormal = intersection.normal; 
    Vector3 lightDir = vector_sub(light.position,hitPoint);
    float albedo = 0.5;
    float r2 = vector3_distance(light.position, hitPoint);
    if (r2 <= 0 || r2 > light.radius) {
        return (Vector3){0,0,0};
    }
    Vector3 lightInt = (Vector3){0.01,0.01,0.01};
    float atenuation = 1 - (r2 / light.radius);
    if (atenuation < 0) {
        atenuation = 0;
    }
    lightInt = vector_scale(lightInt, light.intensity * pow(atenuation,2));
    float val = 0.5/M_PI;
    lightInt.x *= val,lightInt.y *= val,lightInt.z *= val;
    lightInt.x = clamp(lightInt.x, 0.0, 1.0);
    lightInt.y = clamp(lightInt.y, 0.0, 1.0);
    lightInt.z = clamp(lightInt.z, 0.0, 1.0);

    //lightInt.x /= 255,lightInt.y /= 255,lightInt.z /= 255;
    return (lightInt);
}

Vector3 getRandomDirection()
{
    return (Vector3){rand(), rand(), rand()};
}

Vector3 render(Ray ray, Object *objects, int object_count, PointLight *lights, int light_count, Camera cam) {
    Intersection closest_intersection = {0};
    float closest_distance = INFINITY;
    Object touch;
    Vector3 color;
    Vector3 final_light = (Vector3){0,0,0};
    for (int i = 0; i < object_count; i++) {
        Intersection intersection = intersect_ray_object(ray, &objects[i]);
        if (intersection.hit && intersection.distance < closest_distance) {
            closest_distance = intersection.distance;
            closest_intersection = intersection;
            color = objects[i].color;
            touch = objects[i];
        }
    }

    if (!closest_intersection.hit) {
        return (Vector3){0.0f, 0.0f, 1.0f}; // Background color
    }

    Vector3 intensity = {0.1,0.1,0.1}; // Ambient intensity
    int i;
    int light_c = 0;
    for (i = 0; i < light_count; i++) {
        Vector3 light_dir = vector_sub(closest_intersection.point, lights[i].position);
        Ray shadow_ray = {lights[i].position, light_dir};

        if (intersect_ray_object(shadow_ray, &touch).hit) {
            Vector3 ilu = illuminate(ray, closest_intersection, lights[i], touch);
            intensity = vector_add(intensity, ilu);
            intensity.x = clamp(intensity.x, 0.0, 1.0);
            intensity.y = clamp(intensity.y, 0.0, 1.0);
            intensity.z = clamp(intensity.z, 0.0, 1.0);
        }

        //indirect
        /*Vector3 N = closest_intersection.normal;
        Vector3 indirectLight = (Vector3){0, 0, 0};
        float radi = 0;
        int someNumberOfRays = 22;
        for (int i = 0; i < someNumberOfRays; ++i) {
            Vector3 randomDirection = vector_normalize(getRandomDirection());
            for (int j = 0; j < object_count; j++) {
                Ray random_ray = {closest_intersection.point, randomDirection};
                Intersection indirect= intersect_ray_object(random_ray, &objects[j]);
                if (indirect.hit){
                    indirectLight = vector_scale(vector_add(indirectLight, objects[j].color), cos(vector_dot(closest_intersection.normal, light_dir)));
                }
            }
        }
        indirectLight.x /= someNumberOfRays;
        indirectLight.y /= someNumberOfRays;
        indirectLight.z /= someNumberOfRays;*/

    }
    //final_light = vector_add(final_light, vector_scale(color, intensity));
    return intensity; // Placeholder color
}

void	my_mlx_pixel_put(Renderer *data, int x, int y, int color)
{
	char	*dst;

	dst = data->addr + (y * data->line_length + x * (data->bits_per_pixel / 8));
	*(unsigned int*)dst = color;
}
void render_scene(Renderer *renderer, Camera *camera, Object *objects, int object_count, PointLight *lights, int light_count) {
    for (int y = 0; y < renderer->height; y++) {
        for (int x = 0; x < renderer->width; x++) {
            Ray primary_ray = create_primary_ray(camera, x, y, renderer->width, renderer->height);
            Vector3 color = render(primary_ray, objects, object_count, lights, light_count, *camera);
            my_mlx_pixel_put(renderer, x, y, (int)(color.x * 255) << 16 | (int)(color.y * 255) << 8 | (int)(color.z * 255));
        }
    }
}

void quaternion_to_matrix(Quaternion q, float matrix[3][3]) {
    matrix[0][0] = 1 - 2 * q.y * q.y - 2 * q.z * q.z;
    matrix[0][1] = 2 * q.x * q.y - 2 * q.z * q.w;
    matrix[0][2] = 2 * q.x * q.z + 2 * q.y * q.w;

    matrix[1][0] = 2 * q.x * q.y + 2 * q.z * q.w;
    matrix[1][1] = 1 - 2 * q.x * q.x - 2 * q.z * q.z;
    matrix[1][2] = 2 * q.y * q.z - 2 * q.x * q.w;

    matrix[2][0] = 2 * q.x * q.z - 2 * q.y * q.w;
    matrix[2][1] = 2 * q.y * q.z + 2 * q.x * q.w;
    matrix[2][2] = 1 - 2 * q.x * q.x - 2 * q.y * q.y;
}

// Transform a vector by a rotation matrix
Vector3 transform_vector(Vector3 v, float matrix[3][3]) {
    return (Vector3){
        v.x * matrix[0][0] + v.y * matrix[0][1] + v.z * matrix[0][2],
        v.x * matrix[1][0] + v.y * matrix[1][1] + v.z * matrix[1][2],
        v.x * matrix[2][0] + v.y * matrix[2][1] + v.z * matrix[2][2]
    };
}

void quaternion_to_rotation_matrix(const Quaternion *q, float matrix[4][4]) {
    float xx = q->x * q->x;
    float yy = q->y * q->y;
    float zz = q->z * q->z;
    float xy = q->x * q->y;
    float xz = q->x * q->z;
    float yz = q->y * q->z;
    float wx = q->w * q->x;
    float wy = q->w * q->y;
    float wz = q->w * q->z;

    matrix[0][0] = 1.0f - 2.0f * (yy + zz);
    matrix[0][1] = 2.0f * (xy - wz);
    matrix[0][2] = 2.0f * (xz + wy);
    matrix[0][3] = 0.0f;

    matrix[1][0] = 2.0f * (xy + wz);
    matrix[1][1] = 1.0f - 2.0f * (xx + zz);
    matrix[1][2] = 2.0f * (yz - wx);
    matrix[1][3] = 0.0f;

    matrix[2][0] = 2.0f * (xz - wy);
    matrix[2][1] = 2.0f * (yz + wx);
    matrix[2][2] = 1.0f - 2.0f * (xx + yy);
    matrix[2][3] = 0.0f;

    matrix[3][0] = 0.0f;
    matrix[3][1] = 0.0f;
    matrix[3][2] = 0.0f;
    matrix[3][3] = 1.0f;
}

Ray create_primary_ray(const Camera *camera, int x, int y, int width, int height) {
    Ray ray;
    ray.origin = camera->position;

    float aspect_ratio = (float)width / height;
    float fov_rad = camera->fov * (PI / 180.0f);
    float scale = tan(fov_rad * 0.5f) * camera->near_plane;

    float pixel_ndc_x = (x + 0.5f) / (float)width;
    float pixel_ndc_y = (y + 0.5f) / (float)height;

    float pixel_screen_x = (2.0f * pixel_ndc_x - 1.0f) * aspect_ratio * scale;
    float pixel_screen_y = (1.0f - 2.0f * pixel_ndc_y) * scale;

    Vector3 pixel_camera_space = {pixel_screen_x, pixel_screen_y, -camera->near_plane};

    float rotation_matrix[4][4];
    quaternion_to_rotation_matrix(&camera->rotation, rotation_matrix);

    Vector3 ray_direction = {
        rotation_matrix[0][0] * pixel_camera_space.x + rotation_matrix[0][1] * pixel_camera_space.y + rotation_matrix[0][2] * pixel_camera_space.z,
        rotation_matrix[1][0] * pixel_camera_space.x + rotation_matrix[1][1] * pixel_camera_space.y + rotation_matrix[1][2] * pixel_camera_space.z,
        rotation_matrix[2][0] * pixel_camera_space.x + rotation_matrix[2][1] * pixel_camera_space.y + rotation_matrix[2][2] * pixel_camera_space.z
    };

    ray.direction = vector_normalize(ray_direction);
    return ray;
}

void allocate_memory_for_sphere(Object *sphere, int segment_count) {
    int vertex_count = (segment_count + 1) * (segment_count + 1);
    int triangle_count = segment_count * segment_count * 2;

    sphere->vertices = (Vector3*)malloc(sizeof(Vector3) * vertex_count);
    sphere->triangles = (int*)malloc(sizeof(int) * triangle_count * 3);
    // UVs are not used in this example
}

void free_memory_for_sphere(Object *sphere) {
    free(sphere->vertices);
    free(sphere->triangles);
    // Free UVs if allocated
}

void create_sphere_mesh(Object *sphere, float radius, int segment_count) {
    allocate_memory_for_sphere(sphere, segment_count);

    int vertex_count = 0;
    int triangle_count = 0;

    for (int i = 0; i <= segment_count; ++i) {
        for (int j = 0; j <= segment_count; ++j) {
            float theta = (float)i / segment_count * PI;
            float phi = (float)j / segment_count * 2.0f * PI;

            float x = radius * sinf(theta) * cosf(phi);
            float y = radius * cosf(theta);
            float z = radius * sinf(theta) * sinf(phi);

            sphere->vertices[vertex_count++] = (Vector3){x, y, z};
        }
    }

    for (int i = 0; i < segment_count; ++i) {
        for (int j = 0; j < segment_count; ++j) {
            int first = (i * (segment_count + 1)) + j;
            int second = first + segment_count + 1;

            sphere->triangles[triangle_count++] = first;
            sphere->triangles[triangle_count++] = second;
            sphere->triangles[triangle_count++] = first + 1;

            sphere->triangles[triangle_count++] = second;
            sphere->triangles[triangle_count++] = second + 1;
            sphere->triangles[triangle_count++] = first + 1;
        }
    }
    sphere->vert_count = vertex_count;
    sphere->tri_count = triangle_count;
}

void translate_object(Object *object, Vector3 translation) {
    for (int i = 0; i < object->vert_count; ++i) {
        object->vertices[i].x += translation.x;
        object->vertices[i].y += translation.y;
        object->vertices[i].z += translation.z;
    }
}

void create_plane_mesh(Object *plane, float width, float height, int resolution_x, int resolution_z);
void free_plane_mesh(Object *plane);

void create_plane_mesh(Object *plane, float width, float height, int resolution_x, int resolution_z) {
    int vertex_count = (resolution_x + 1) * (resolution_z + 1);
    int triangle_count = resolution_x * resolution_z * 2;
    plane->tri_count = triangle_count;
    plane->vert_count = vertex_count;
    plane->vertices = (Vector3*)malloc(sizeof(Vector3) * vertex_count);
    plane->triangles = (int*)malloc(sizeof(int) * triangle_count * 3);

    float half_width = width / 2.0f;
    float half_height = height / 2.0f;
    float dx = width / resolution_x;
    float dz = height / resolution_z;

    int vertex_index = 0;
    for (int z = 0; z <= resolution_z; ++z) {
        for (int x = 0; x <= resolution_x; ++x) {
            plane->vertices[vertex_index++] = (Vector3){
                x * dx - half_width,
                0,
                z * dz - half_height
            };
        }
    }

    int triangle_index = 0;
    for (int z = 0; z < resolution_z; ++z) {
        for (int x = 0; x < resolution_x; ++x) {
            int top_left = z * (resolution_x + 1) + x;
            int top_right = top_left + 1;
            int bottom_left = top_left + (resolution_x + 1);
            int bottom_right = bottom_left + 1;

            // First triangle
            plane->triangles[triangle_index++] = top_left;
            plane->triangles[triangle_index++] = bottom_left;
            plane->triangles[triangle_index++] = top_right;

            // Second triangle
            plane->triangles[triangle_index++] = top_right;
            plane->triangles[triangle_index++] = bottom_left;
            plane->triangles[triangle_index++] = bottom_right;
        }
    }
}

void free_plane_mesh(Object *plane) {
    free(plane->vertices);
    free(plane->triangles);
}



int main() {
    Renderer renderer;
    renderer.width = 800;
    renderer.height = 600;
    renderer.mlx = mlx_init();
    renderer.win = mlx_new_window(renderer.mlx, renderer.width, renderer.height, "Ray Tracer");
    renderer.img = mlx_new_image(renderer.mlx, 800, 600);
    renderer.addr = mlx_get_data_addr(renderer.img, &renderer.bits_per_pixel, &renderer.line_length,
								&renderer.endian);
    Camera camera = {{0, 2, 4}, {-0.247404, 0, 0, 0.9689124 }, 0.001f, 100.0f, 90.0f};
    Object sphere;
    create_sphere_mesh(&sphere, 1, 12);
    sphere.color = (Vector3){0,1,0};
    Object plane;
    plane.color = (Vector3){1,0,0};
    create_plane_mesh(&plane, 20, 20, 2, 2);
    translate_object(&plane, (Vector3){5,-1,5});
    Object objects[] = {sphere, plane};
    int object_count = sizeof(objects) / sizeof(objects[0]);
    PointLight light = {{-1.5, 0.1, 1.5}, 2.0f, 5000.0f};
    PointLight light2 = {{1.5, 0.1, 1.5}, 10.0f, 500.0f};
    PointLight lights[] = {light, light2};
    int light_count = sizeof(lights) / sizeof(lights[0]);

    render_scene(&renderer, &camera, objects, object_count, lights, light_count);

    mlx_put_image_to_window(renderer.mlx, renderer.win, renderer.img, 0, 0);
    mlx_loop(renderer.mlx);
    free(renderer.framebuffer);
    return 0;
}