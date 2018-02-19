<%@ include file="header.jsp" %>
<%@ include file="sidebar.jsp" %>
<sql:query dataSource="${ds}" var="result">
SELECT * from category;
</sql:query> 
<section id="main-content">
<section class="wrapper">
<font color="red" ><strong><c:out value="${msg }"></c:out></strong></font>
 <div class="row">
                  <div class="col-lg-12">
                      <section class="panel">
                          <header class="panel-heading">
                              Manage Category                          </header>
                          
                          <table class="table table-striped table-advance table-hover">
                           <tbody>
                              <tr>
                                 <th width="45%"><i class="icon_profile"></i> Category Name </th>
                                 <th width="55%"><i class="icon_cogs"></i> Action</th>
                              </tr>
                              <c:forEach var="row" items="${result.rows}">
                              <tr>
                                 <td><c:out value="${row.name}"/></td>
                                 <td>
                                  <div class="btn-group">
                                      <a class="btn btn-primary" href="<%=request.getContextPath() %>/Admin?action=category-edit&id=<c:out value="${row.id}"/>"><i class="icon_plus_alt2"></i></a>
                                      <a class="btn btn-danger" href="<%=request.getContextPath() %>/Admin?action=category-delete&id=<c:out value="${row.id}"/>"><i class="icon_close_alt2"></i></a>                                  </div>                                </td>
                              </tr>  
                              </c:forEach>                            
                           </tbody>
                        </table>
                      </section>
                  </div>
              </div>
</section>
</section>
<%@ include file="footer.jsp" %>