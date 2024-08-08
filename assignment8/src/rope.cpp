#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

//        Comment-in this part when you implement the constructor
//        for (auto &i : pinned_nodes) {
//            masses[i]->pinned = true;
//        }
        Vector2D step = (end - start) / (num_nodes - 1);
        for(int i = 0; i < num_nodes; i++){
            masses.push_back(new Mass(start + step * i, node_mass, true));
            if(i){
                springs.push_back(new Spring(masses[i-1], masses[i], k));
                masses[i]->pinned = false;
            }
        }
        
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            float length = (s->m2->position-s->m1->position).norm();
            Vector2D dir = (s->m2->position-s->m1->position);
            Vector2D force = s->k * (length-s->rest_length) * dir/length;  // 由m1->m2的力
            s->m1->forces+=force;
            s->m2->forces-=force;

            // damping
            Vector2D reve = s->m2->velocity - s->m1->velocity;
            Vector2D force1 = 0.05 * (reve.x * dir.x + reve.y * dir.y)/length * dir/length;
            s->m1->forces+=force1;
            s->m2->forces-=force1;

            // air damping
            // 欧拉法Global Damping force的计算就是 f_d = -k_d * v
            s->m1->forces -= 0.005 *s->m1->velocity;
            s->m2->forces -= 0.005 *s->m2->velocity;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces +=gravity * m->mass;
                m->velocity += m->forces/m->mass * delta_t;
                m->position += m->velocity * delta_t;  // 使用的velocity 是v(t+1),所以是半隐式欧拉
            //    m->velocity += m->forces/m->mass * delta_t;

                // TODO (Part 2): Add global damping
                // global damping已经在上面的for循环计算过了
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            float length = (s->m2->position-s->m1->position).norm();
            Vector2D dir = (s->m2->position-s->m1->position)/length; // m1 --> m2

            if( s->m1->pinned && s->m2->pinned) continue;
            if( s->m1->pinned && !s->m2->pinned) 
            {
                s->m2->position -= (length - s->rest_length) * dir;
            }
            else if(!s->m1->pinned && s->m2->pinned)
            {
                s->m1->position += (length - s->rest_length) * dir;
            }
            else // ! m1.pinned && !m2.pinned 
            {
                s->m1->position += (length - s->rest_length) * dir * 0.5f;
                s->m2->position -= (length - s->rest_length) * dir * 0.5f;
            }
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->forces += gravity * m->mass;
                Vector2D a = m->forces / m->mass;
                constexpr float damping_coefficient=0.00005;
                m->position = m->position + (1 - damping_coefficient) * (m->position - m->last_position) + a * delta_t * delta_t;
                m->last_position = temp_position;
                
                // TODO (Part 4): Add global Verlet damping
            }
            m->forces = Vector2D(0,0);
        }
    }
}
