#ifndef __BAG_RECORDER__
#define __BAG_RECORDER__


#include <sys/types.h>

#include <mutex>
#include <string>
#include <vector>


#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>

class BagRecorder
{
public:
    bool start()
    {
        if (pid_ > 0)
            return false;

        pid_ = fork();

        if (pid_ < 0)
        {
            pid_ = -1;
            return false;
        }

        if (pid_ == 0)
        {
            // Processo filho

            setpgid(0, 0);

            execlp(
                "ros2",
                "ros2",
                "bag",
                "record",
                "-a",
                nullptr
            );

            // Só chega aqui se execlp falhar
            _exit(1);
        }

        return true;
    }

    bool stop()
    {
        if (pid_ <= 0)
            return false;

        // Equivale ao Ctrl+C
        kill(-pid_, SIGINT);

        // Espera o rosbag terminar corretamente
        waitpid(pid_, nullptr, 0);

        pid_ = -1;

        return true;
    }

    bool isRecording() const
    {
        return pid_ > 0;
    }

    ~BagRecorder()
    {
        stop();
    }

private:
    pid_t pid_ = -1;
};

#endif