/* ============================================================
 *
 * This file is a part of the RSB project
 *
 * Copyright (C) 2011 by Johannes Wienke <jwienke at techfak dot uni-bielefeld dot de>
 *
 * This file may be licensed under the terms of the
 * GNU Lesser General Public License Version 3 (the ``LGPL''),
 * or (at your option) any later version.
 *
 * Software distributed under the License is distributed
 * on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
 * express or implied. See the LGPL for the specific language
 * governing rights and limitations.
 *
 * You should have received a copy of the LGPL along with this
 * program. If not, go to http://www.gnu.org/licenses/lgpl.html
 * or write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The development of this software was supported by:
 *   CoR-Lab, Research Institute for Cognition and Robotics
 *     Bielefeld University
 *
 * ============================================================ */

#include <iostream>

#include <stdlib.h>
#include <math.h>

#include <boost/format.hpp>
#include <boost/program_options.hpp>

#include <rsc/threading/PeriodicTask.h>
#include <rsc/threading/ThreadedTaskExecutor.h>

#include <rsb/Informer.h>
#include <rsb/Factory.h>

using namespace std;
using namespace rsc::misc;
using namespace rsb;
namespace po = boost::program_options;

class FloodingTask: public rsc::threading::PeriodicTask {
public:

    FloodingTask(Informer<string>::Ptr informer, const unsigned int& ms = 2,
            const string& payload = "") :
            rsc::threading::PeriodicTask(ms), iteration(1), informer(informer), payload(
                    payload) {
    }

    virtual ~FloodingTask() {
    }

    virtual void execute() {

        string temp = boost::str(
                boost::format("this is iteration %d") % iteration++) + payload;
        boost::shared_ptr<string> data(new string(temp));
        informer->publish(data);

    }

private:

    unsigned long long iteration;
    Informer<string>::Ptr informer;
    string payload;

};

int main(int argc, char* argv[]) {

    po::options_description desc("Allowed options");
    desc.add_options()("int,i", po::value<unsigned int>(), "interval in ms")(
            "scope,s", po::value<string>(), "destination scope")("help,h",
            "help message")("payload,p", po::value<unsigned int>(),
            "Add additional payload of specified size (bytes)");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        cout << desc << "\n";
        return EXIT_SUCCESS;
    }

    po::notify(vm);

    unsigned int intervalMs = vm["int"].as<unsigned int>();
    Scope scope = vm["scope"].as<string>();

    string payload;
    if (vm.count("payload")) {
        payload = rsc::misc::randAlnumStr(vm["payload"].as<unsigned int>());
        cout << "Generated payload of size " << payload.size() << endl;
    }

    cout << "sending on " << scope << " with interval " << intervalMs << endl;

    // ---

    Factory& factory = getFactory();

    Informer<string>::Ptr informer = factory.createInformer<string>(scope);

    rsc::threading::ThreadedTaskExecutor exec;

    exec.schedule(
            rsc::threading::TaskPtr(new FloodingTask(informer, intervalMs, payload)));

    while (true) {
        boost::this_thread::sleep(boost::posix_time::seconds(1000));
    }

    return EXIT_SUCCESS;

}
