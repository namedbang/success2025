/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-03 14:38:01
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-03 15:06:10
 * @FilePath: /success2025/src/app/api/Observer.hpp
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */
#ifndef __OBSERVER_H__
#define __OBSERVER_H__

#include <memory>
#include <iostream>
#include <list>
#include <algorithm>

class Observer;
// 抽象被观察者
class Subject
{
public:
    Subject() : m_nState(0) {}

    virtual ~Subject() = default;

    virtual void Attach(const std::shared_ptr<Observer> pObserver) = 0;

    virtual void Detach(const std::shared_ptr<Observer> pObserver) = 0;

    virtual void Notify() = 0;

    virtual int GetState() { return m_nState; }

    void SetState(int state)
    {
        std::cout << "Subject updated !" << std::endl;
        m_nState = state;
    }

protected:
    std::list<std::shared_ptr<Observer>> m_pObserver_list;
    int m_nState;
};

// 抽象观察者
class Observer
{
public:
    virtual ~Observer() = default;

    Observer(const std::shared_ptr<Subject> pSubject, const std::string &name = "unknown")
        : m_pSubject(pSubject), m_strName(name) {}

    virtual void Update() = 0;

    virtual const std::string &name() { return m_strName; }

protected:
    std::shared_ptr<Subject> m_pSubject;
    std::string m_strName;
};

// 具体被观察者
class ConcreteSubject : public Subject
{
public:
    void Attach(const std::shared_ptr<Observer> pObserver) override
    {
        auto iter = std::find(m_pObserver_list.begin(), m_pObserver_list.end(), pObserver);
        if (iter == m_pObserver_list.end())
        {
            std::cout << "Attach observer" << pObserver->name() << std::endl;
            m_pObserver_list.emplace_back(pObserver);
        }
    }

    void Detach(const std::shared_ptr<Observer> pObserver) override
    {
        std::cout << "Detach observer" << pObserver->name() << std::endl;
        m_pObserver_list.remove(pObserver);
    }

    // 循环通知所有观察者
    void Notify() override
    {
        auto it = m_pObserver_list.begin();
        while (it != m_pObserver_list.end())
        {
            (*it++)->Update();
        }
    }
};

// //具体观察者1
// class Observer1 : public Observer {
// public:
//     Observer1(const std::shared_ptr<Subject> pSubject, const std::string &name = "unknown")
//             : Observer(pSubject, name) {}

//     void Update() override {
//         std::cout << "Observer1_" << m_strName << " get the update.New state is: "
//                   << m_pSubject->GetState() << std::endl;
//     }
// };

#endif /* __OBSERVER_H__ */
