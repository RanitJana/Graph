#include <bits/stdc++.h>
using namespace std;

namespace functions
{
    void bfs(int start, int V, vector<int> adj[], int *vis)
    {
        queue<int> q;
        q.push(start);
        vis[start] = 1;
        while (q.size())
        {
            int node = q.front();
            q.pop();
            cout << node << " ";
            for (auto adjNode : adj[node])
            {
                if (!vis[adjNode])
                {
                    vis[adjNode] = 1;
                    q.push(adjNode);
                }
            }
        }
    }

    void dfs(int node, vector<int> adj[], int *vis)
    {
        vis[node]++;
        cout << node << " ";
        for (auto adjNode : adj[node])
        {
            if (!vis[adjNode])
                dfs(adjNode, adj, vis);
        }
    }

    bool checkCycleUsingDfs(int prev, int node, vector<bool> &vis, vector<int> adj[])
    {
        vis[node] = true;
        for (auto it : adj[node])
        {
            if (vis[it] && prev != it)
                return true;
            if (!vis[it] && checkCycleUsingDfs(node, it, vis, adj))
                return true;
        }
        return false;
    }

    bool colorAndCheckBipartite(int start, int color[], vector<int> adj[])
    {
        color[start] = 1;
        queue<int> q;
        q.push(start);
        while (!q.empty())
        {
            int node = q.front();
            q.pop();
            for (auto it : adj[node])
            {
                if (color[it] == -1)
                {
                    color[it] = !color[node];
                    q.push(it);
                }
                else if (color[it] == color[node])
                    return false;
            }
        }
        return true;
    }

    void findTopo(int node, vector<int> adj[], vector<bool> &vis, stack<int> &s)
    {
        vis[node] = true;
        for (auto it : adj[node])
        {
            if (!vis[it])
            {
                findTopo(it, adj, vis, s);
            }
        }
        s.push(node);
    }
}

void bfs(int V, vector<int> adj[]) // 1based
{
    int vis[V + 1] = {0};
    for (int i = 1; i <= V; i++)
    {
        if (!vis[i])
        {
            functions::bfs(i, V, adj, vis);
        }
    }
}

void dfs(int V, vector<int> adj[]) // 1based
{
    int vis[V + 1] = {0};
    for (int i = 1; i <= V; i++)
    {
        if (!vis[i])
        {
            functions::dfs(i, adj, vis);
        }
    }
}

bool isCyclePresentBfs(int V, vector<int> adj[]) // 1based
{
    int parent[V + 1];
    memset(parent, -1, sizeof(parent));

    queue<int> q;
    q.push(1);

    parent[1] = -2;

    while (!q.empty())
    {
        int node = q.front();
        q.pop();
        for (auto adjNode : adj[node])
        {
            if (parent[adjNode] == -1)
            {
                q.push(adjNode);
                parent[adjNode] = node;
            }
            else if (parent[node] != adjNode)
            {
                return true;
            }
        }
    }
    return false;
}

bool isCyclePresentDfs(int V, vector<int> adj[]) // 1based
{
    vector<bool> vis(V + 1, false);
    for (int i = 1; i <= V; i++)
    {
        if (!vis[i] && functions::checkCycleUsingDfs(-1, i, vis, adj))
            return true;
    }
    return false;
}
bool isBipartite(int V, vector<int> adj[]) // 1 based
{
    int color[V + 1];
    memset(color, -1, sizeof(color));
    for (int i = 1; i <= V; i++)
    {
        if (color[i] == -1 && !functions::colorAndCheckBipartite(i, color, adj))
            return false;
    }
    return true;
}

void topoSortDfs(int V, vector<int> adj[]) // 0 based
{
    vector<bool> vis(V, false);
    stack<int> s;
    for (int i = 0; i < V; i++)
    {
        if (!vis[i])
        {
            functions::findTopo(i, adj, vis, s);
        }
    }
    cout << "Topological Sort is : ";
    while (!s.empty())
    {
        cout << s.top() << " ";
        s.pop();
    }
    cout << endl;
}

void topoSortKhanAlgo(int V, vector<int> adj[])
{
    int indegree[V] = {0};
    for (int i = 0; i < V; i++)
    {
        for (auto adjNode : adj[i])
        {
            indegree[adjNode]++;
        }
    }
    queue<int> q;
    for (int i = 0; i < V; i++)
    {
        if (indegree[i] == 0)
            q.push(i);
    }
    cout << "Topological Sort is : ";
    while (!q.empty())
    {
        int node = q.front();
        cout << node << " ";
        q.pop();
        for (int adjNode : adj[node])
        {
            indegree[adjNode]--;
            if (indegree[adjNode] == 0)
                q.push(adjNode);
        }
    }
    cout << endl;
}

int main()
{
    /*int E, V;
    cout << "Enter Edges and Vertices repectively : ";
    cin >> E >> V;

    vector<int> adj[V + 1];

    for (int i = 0; i < E; i++)
    {
        int u, v;
        cout << "Enter two vertices : ";
        cin >> u >> v;
        adj[u].push_back(v);
        adj[v].push_back(u);
    }
    bfs(V, adj);
    cout << endl;
    dfs(V, adj);
    cout << endl;
    cout << isCyclePresentBfs(V, adj) << " " << isCyclePresentDfs(V, adj) << " " << isBipartite(V, adj);
    */

    int V, E;
    cin >> V >> E;
    vector<int> adj[V];
    for (int i = 0; i < E; i++)
    {
        int u, v;
        cin >> u >> v;
        adj[u].push_back(v);
    }
    topoSortDfs(V, adj);
    topoSortKhanAlgo(V, adj);
    return 0;
}