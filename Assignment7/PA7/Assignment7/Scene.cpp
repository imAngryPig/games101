//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){ // 判断是否是光源,是则继续
            emit_area_sum += objects[k]->getArea();   // 获得所有光源面积
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){    
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){            // 在所有光源的各个分光源随机选一个进行采样
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // // TO DO Implement Path Tracing Algorithm here
    // Intersection inter=intersect(ray);
    // if(!inter.happened) return Vector3f();

    // // if hit the light, return the light emission
    // if(inter.m->hasEmission()){
    //     return inter.m->getEmission();
    // }

    // Vector3f l_dir(0.0f);
    // Vector3f l_indir(0.0f);

    // Intersection lightInter;
    // float lightpdf=0.0f;
    // sampleLight(lightInter,lightpdf);

    // Vector3f obj2light = (lightInter.coords-inter.coords);
    // Vector3f obj2lightDir = obj2light.normalized();
    // float obj2light_pow = obj2light.x * obj2light.x + obj2light.y * obj2light.y + obj2light.z * obj2light.z; // 击中点到采样光点的距离模的平方

    // // 接下来判 这段光路中间是否有东西遮挡

    // Ray obj2lightRay(inter.coords,obj2lightDir);
    // Intersection shadow = intersect(obj2lightRay);
    // // 在没有遮挡的情况下,计算l_dir
    // if( shadow.distance - obj2light.norm()  > -EPSILON*1000 && lightpdf-0.0f >EPSILON)
    // {
    //     l_dir = lightInter.emit * inter.m->eval(ray.direction,obj2lightDir,inter.normal) 
    //             * dotProduct(obj2lightDir,inter.normal)
    //             * dotProduct(-obj2lightDir,lightInter.normal)
    //             / obj2light_pow / lightpdf;
    // }

    // // RR 掐掉没通过测试的光线
    // if(get_random_float()>RussianRoulette) return l_dir;

    // // 开始计算l_indir
    // Vector3f outDir = inter.m->sample(ray.direction,inter.normal).normalized();
    // Ray outRay(inter.coords,outDir);
    // Intersection reflectInter = intersect(outRay);
    // if(reflectInter.happened && !reflectInter.m->hasEmission() && inter.m->pdf(ray.direction,outDir,inter.normal)>EPSILON )
    // {
    //     l_indir = castRay(outRay,depth+1) * inter.m->eval(ray.direction,outDir,inter.normal)
    //               * dotProduct(outDir,inter.normal) // cos(θ)
    //               / inter.m->pdf(ray.direction,outDir,inter.normal)
    //               / RussianRoulette;
    // }
    // return l_dir + l_indir;


    Vector3f L_dir = {0, 0, 0}, L_indir = {0, 0, 0};
    Intersection intersection = Scene::intersect(ray); //求一条光线与场景的交点
    if (!intersection.happened) //没交点
        return {};
    if (intersection.m->hasEmission()) //一、交点是光源：
        return intersection.m->getEmission();
    /*
    bool Material::hasEmission() {
        if (m_emission.norm() > EPSILON) return true; 这里只有光才＞0
        else return false;
    }
    */
    //---------二、交点是物体：1)向光源采样计算direct----------
    Intersection lightpos;
    float lightpdf = 0.0f;
    sampleLight(lightpos, lightpdf);//获得对光源的采样，包括光源的位置和采样的pdf(在场景的所有光源上按面积 uniform 地 sample 一个点，并计算该 sample 的概率密度)
    Vector3f collisionlight = lightpos.coords - intersection.coords;
    float dis = dotProduct(collisionlight, collisionlight);
    Vector3f collisionlightdir = collisionlight.normalized();   
    Ray light_to_object_ray(intersection.coords, collisionlightdir);
    Intersection light_to_anything_ray = Scene::intersect(light_to_object_ray);
    auto f_r = intersection.m -> eval(ray.direction, collisionlightdir, intersection.normal);
    if (light_to_anything_ray.distance - collisionlight.norm() > -0.005){  //没有遮挡,有横条就是数太小了！
        //L_dir = L_i * f_r * cos_theta * cos_theta_x / |x - p | ^ 2 / pdf_light
        L_dir = lightpos.emit * f_r * dotProduct(collisionlightdir, intersection.normal) * dotProduct(-collisionlightdir, lightpos.normal) / dis / lightpdf;
    }

    //--------二、交点是物体：2)向其他物体采样递归计算indirect---------
    if (get_random_float() > RussianRoulette)     //打到物体后对半圆随机采样使用RR算法
        return L_dir;
    Vector3f w0 = intersection.m -> sample(ray.direction, intersection.normal).normalized();
    Ray object_to_object_ray(intersection.coords, w0);
    Intersection islight = Scene::intersect(object_to_object_ray);
    if (islight.happened && !islight.m->hasEmission())
    {   // shade(q, wi) * f_r * cos_theta / pdf_hemi / P_RR
        float pdf = intersection.m->pdf(ray.direction, w0, intersection.normal);
        f_r = intersection.m->eval(ray.direction, w0, intersection.normal);
        L_indir = castRay(object_to_object_ray, depth + 1) * f_r * dotProduct(w0, intersection.normal) / pdf / RussianRoulette;
    }
    return L_dir + L_indir;


}